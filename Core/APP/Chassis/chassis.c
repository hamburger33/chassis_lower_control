#include "chassis.h"
#include "pub_sub.h"

#include <arm_math.h>
#include <math.h>

#include "bsp.h"
#include "bsp_random.h"
#include "common.h"
#include "controller.h"

void chassis_motor_lost(void *motor) { printf_log("chassis motor lost!\n"); }
void chassis_imu_lost(void *motor) { printf_log("chassis IMU lost!!\n"); }
void chassis_super_cap_lost(void *motor) { printf_log("super cap lost!!\n"); }

// 静态保存 Chassis_Create 注册的 Publisher
static Publisher *s_chassis_cmd_pub = NULL;

// UART 接收回调——把 CAN 帧转成 Cmd_chassis_t 并 publish
static void Chassis_UARTRxCallback(uint8_t uart_index, uint8_t *data, uint32_t len) {
    // 只处理指定 UART 和长度正确的数据
    if (uart_index != UART_INDEX) return;  // 替换 UART_INDEX 为实际的 UART 索引
    if (len != sizeof(Cmd_chassis)) return;  // 确保接收到的数据长度符合 Cmd_chassis 的大小

    // 将接收到的数据复制到 Cmd_chassis 结构体
    Cmd_chassis cmd;
    memcpy(&cmd, data, sizeof(cmd));

    // 将数据封装到 publish_data 结构体中
    publish_data pd = {
        .data = (uint8_t *)&cmd,
        .len  = sizeof(cmd)
    };

    // 发布数据到相应的发布者
    s_chassis_cmd_pub->publish(s_chassis_cmd_pub, pd);
}

Chassis *Chassis_Create() {
    Chassis *obj = (Chassis *)malloc(sizeof(Chassis));
    memset(obj, 0, sizeof(Chassis));

    obj->offset_x = CHASSIS_ROTATE_X_OFFSET;
    obj->offset_y = CHASSIS_ROTATE_Y_OFFSET;

    // 超级电容
    super_cap_wuli_config cap_config;
    cap_config.bsp_can_index = 0;
    cap_config.super_cap_wuli_rx_id = SUPER_CAP_WULI_RX_ID;
    cap_config.super_cap_wuli_tx_id = SUPER_CAP_WULI_TX_ID;
    cap_config.lost_callback = chassis_super_cap_lost;
    obj->super_cap = Super_cap_wuli_Create(&cap_config);

    // 底盘机构参数初始化
    can_motor_config lf_config;
    can_motor_config rf_config;
    can_motor_config lb_config;
    can_motor_config rb_config;
    controller_config lf_controller_config;
    controller_config rf_controller_config;
    controller_config lb_controller_config;
    controller_config rb_controller_config;
    // lf
    memset(&lf_controller_config, 0, sizeof(controller_config));
    CLASSIC_PID_Setconfig(&lf_controller_config, 4.5, 3, 0, 0, 15000, 0, 0);
    lf_config.motor_model = MODEL_3508;
    lf_config.bsp_can_index = 0;
    lf_config.motor_set_id = 1;
    lf_config.motor_controller_config = lf_controller_config;
    lf_config.position_fdb_model = MOTOR_FDB;
    lf_config.speed_fdb_model = MOTOR_FDB;
    lf_config.output_model = MOTOR_OUTPUT_NORMAL;
    lf_config.lost_callback = chassis_motor_lost;
    obj->lf = Can_Motor_Create(&lf_config);

    // lb
    memset(&lb_controller_config, 0, sizeof(controller_config));
    CLASSIC_PID_Setconfig(&lb_controller_config, 4, 2, 0, 0, 15000, 0, 0);
    lb_config.motor_model = MODEL_3508;
    lb_config.bsp_can_index = 0;
    lb_config.motor_set_id = 2;
    lb_config.motor_controller_config = lb_controller_config;
    lb_config.position_fdb_model = MOTOR_FDB;
    lb_config.speed_fdb_model = MOTOR_FDB;
    lb_config.output_model = MOTOR_OUTPUT_NORMAL;
    lb_config.lost_callback = chassis_motor_lost;
    obj->lb = Can_Motor_Create(&lb_config);

    // rb
    memset(&rb_controller_config, 0, sizeof(controller_config));
    CLASSIC_PID_Setconfig(&rb_controller_config, 4.5, 2, 0, 0, 15000, 0, 0);
    rb_config.motor_model = MODEL_3508;
    rb_config.bsp_can_index = 0;
    rb_config.motor_set_id = 3;
    rb_config.motor_controller_config = rb_controller_config;
    rb_config.position_fdb_model = MOTOR_FDB;
    rb_config.speed_fdb_model = MOTOR_FDB;
    rb_config.output_model = MOTOR_OUTPUT_NORMAL;
    rb_config.lost_callback = chassis_motor_lost;
    obj->rb = Can_Motor_Create(&rb_config);

    // rf
    memset(&rf_controller_config, 0, sizeof(controller_config));
    CLASSIC_PID_Setconfig(&rf_controller_config, 4, 3, 0, 0, 15000, 0, 0);
    rf_config.motor_model = MODEL_3508;
    rf_config.bsp_can_index = 0;
    rf_config.motor_set_id = 4;
    rf_config.motor_controller_config = rf_controller_config;
    rf_config.position_fdb_model = MOTOR_FDB;
    rf_config.speed_fdb_model = MOTOR_FDB;
    rf_config.output_model = MOTOR_OUTPUT_NORMAL;
    rf_config.lost_callback = chassis_motor_lost;
    obj->rf = Can_Motor_Create(&rf_config);
    
    /*此处问题在于如何传入目标速度以及传出实际速度*/
    //运动学解算给电机赋值
    float set_speedL;
    float set_speedR;
    float Vcx = 40;//给入目标速度与角速度，前进和逆时针为正方向
    float Wc = 60;
    float C = 0.5;
    float r = 0.1;
    set_speedL= (2*Vcx-Wc*C)/(2*r);//ref_speed单位是m/s
    set_speedR= (2*Vcx+Wc*C)/(2*r);

    //下面部分直接对四个轮子进行赋值
    // //下面部分用运动学分析计算各个电机的ref_speed预期速度+ros键盘控制
    obj->lf->enable = MOTOR_ENABLE;
    obj->lf->motor_controller->ref_speed = set_speedL;
    obj->lb->enable = MOTOR_ENABLE;
    obj->lb->motor_controller->ref_speed = set_speedL;
    obj->rf->enable = MOTOR_ENABLE;
    obj->rf->motor_controller->ref_speed = -set_speedR;
    obj->rb->enable = MOTOR_ENABLE;
    obj->rb->motor_controller->ref_speed = -set_speedR;

    //根据实际速度反解
    float Vcx1;
    float Wc1;
    float left_wheel_speed = obj->lb->fdbSpeed/19.0f/9.55f;//rpm转换单位至rad/s
    float right_wheel_speed = obj->rb->fdbSpeed/19.0f/9.55f;//此处将前后轮视作相等，后续优化可以将前后轮数据处理
    float vL = left_wheel_speed * r;
    float vR = right_wheel_speed * r;
    Vcx1 = 0.5*vL + 0.5*vR; //rad/s-->m/s
    Wc1 = -vL/C+vR/C;

    // 注册 pub/sub
    s_chassis_cmd_pub      = register_pub(CHASSIS_CMD_TOPIC);
    obj->chassis_cmd_suber = register_sub(CHASSIS_CMD_TOPIC, 1);
    obj->chassis_status_pub = register_pub(CHASSIS_STATUS_TOPIC);

    // 注册 UART回调
    BSP_UART_RegisterRxCallback(UART_INDEX, Chassis_UARTRxCallback);

    return obj;
}

void Chassis_Update(Chassis *obj)
{
    // 写运动指令
    // publish_data chassis_data = obj->gimbal_cmd_sub->getdata(obj->chassis_cmd_sub);
    // if (chassis_data.len == -1) return;  // cmd未工作
    // obj->cmd_data = (Cmd_chassis *)chassis_data.data;
    // obj->chassis3508->enable = MOTOR_ENABLE;
    // obj->chassis3508->motor_controller->ref_speed = obj->cmd_data->vx;

    publish_data cmd_pd = obj->chassis_cmd_suber->getdata(obj->chassis_cmd_suber);
    if (cmd_pd.len == sizeof(Cmd_chassis)) {
        obj->cmd_data = (Cmd_chassis *)cmd_pd.data;
    }

    float set_speedL;
    float set_speedR;
    float Vcx = obj->cmd_data->vx;
    float Wc = obj->cmd_data->wz;
    float C = 0.5;
    float r = 0.1;
    set_speedL= (2*Vcx-Wc*C)/(2*r);//ref_speed单位是m/s
    set_speedR= (2*Vcx+Wc*C)/(2*r);

    // 使能电机并写入目标速度到 motor_controller->ref_speed
    obj->lf->enable = MOTOR_ENABLE;
    obj->lb->enable = MOTOR_ENABLE;
    obj->rf->enable = MOTOR_ENABLE;
    obj->rb->enable = MOTOR_ENABLE;

    obj->lf->motor_controller->ref_speed =  set_speedL;
    obj->lb->motor_controller->ref_speed =  set_speedL;
    obj->rf->motor_controller->ref_speed = -set_speedR;  // 右侧电机反向
    obj->rb->motor_controller->ref_speed = -set_speedR;
}
