//此文件为CAN电机的驱动文件，其执行逻辑如下：
//在主控注册了一个电机后，freertos中can_motor线程会周期执行发送电机控制的函数（Can_Motor_Calc_Send），而电机信息对主控的反馈是周期自动进行的（中断后执行CanMotor_RxCallBack实现数据更新），通过调整线程执行周期实现这两步骤的
//电机的创建与注册:注册驱动 Can_Motor_Driver_Init   --->   Can_Motor_Create
//数据更新：CanMotor_RxCallBack  -->  Can_Motor_FeedbackData_Update
//控制数据发送: Can_Motor_Calc_Send  -->  Can_Motor_Send

#include "can_motor.h"

#include "bsp_can.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "string.h"

#define VELOCITY_WINDOW 5
#define FDB_INIT_VALUE (-10000)

can_motor *motor_instances[2][3][4];  // motor_instances用于存放can_motor*类型的指向电机实体的指针，在每个电机初始化时被填充  //注：此处电机对象的存放方式与其他外设不太一样，由于电机有不同的类型和不同的通讯线（can1和can2），便将注册的不同电机分类存放
uint8_t motors_id[2][3][8];           // motors_id
                                      // 被填充为1表示此处有电机被注册，为0表示此处没有电机被注册
const uint32_t identifiers[3] = {0x200, 0x1FF, 0x2FF};
void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len);
void Can_Motor_FeedbackData_Update(can_motor *obj, uint8_t *data);
void Can_Motor_Send(uint8_t can_id, uint16_t identifier, short motor_data_id1, short motor_data_id2, short motor_data_id3, short motor_data_id4);

void Can_Motor_Driver_Init() {
    memset(&motor_instances, 0, sizeof(motor_instances));
    memset(&motors_id, 0, sizeof(motors_id));  //全部初始化
    BSP_CAN_RegisterRxCallback(0, CanMotor_RxCallBack);
    BSP_CAN_RegisterRxCallback(1, CanMotor_RxCallBack);  //驱动注册，注册中断回调函数--主控收到电机回调的数据后会自动调用注册的函数--CanMotor_RxCallBack
}

can_motor *Can_Motor_Create(can_motor_config *config) {  //创建电机函数
    can_motor *obj = (can_motor *)malloc(sizeof(can_motor));
    memset(obj, 0, sizeof(can_motor));
    obj->position_queue = create_circular_queue(sizeof(float), VELOCITY_WINDOW);  //循环队列，用于计算电机转过的总长度，从而确定电机转动的信息
    obj->config = *config;                                                        //写入电机注册配置
    obj->position_sum = 0;                                                        //循环队列数值总和赋初值
    obj->fdbPosition = FDB_INIT_VALUE;                                            //编码器反馈赋初值
    if (obj->config.speed_fdb == NULL) {
        obj->config.speed_fdb = &obj->velocity;  //若速度环不需要其他反馈，则使用自身速度反馈
    }
    if (obj->config.position_fdb == NULL) {  //若位置环不需要其他反馈，则使用自身位置反馈
        obj->config.position_fdb = &obj->real_position;
    }
    obj->motor_controller =  create_controller(&(obj->config.motor_controller_config));
    obj->monitor = Monitor_Register(obj->config.lost_callback, 5, obj);
    if (obj->motor_controller->leso.config.SWITCH == MYENABLE) {
        obj->motor_controller->leso.config.p_input = &(obj->electric_current);
    }

    obj->enable = MOTOR_STOP;
    motors_id[obj->config.bsp_can_index][obj->config.motor_model][obj->config.motor_set_id - 1] = 1;  //根据电机注册配置的信息将存放电机的区域对应位置填入电机注册的信息，并表明有电机注册
    switch (config->motor_model) {
        case MODEL_2006:
            if (config->motor_set_id < 5) {
                motor_instances[config->bsp_can_index][0][config->motor_set_id - 1] = obj;  //若为2006电机，存放在motor_instances数组中第0行，四个位置对应电机id 1-4
            } else {
                motor_instances[config->bsp_can_index][1][config->motor_set_id - 5] = obj;  //若为2006电机，存放在motor_instances数组中第1行，四个位置对应电机id 5-8
            }
            BSP_CAN_AddFilter(obj->config.bsp_can_index, 0x200 + config->motor_set_id);  //写入can的过滤器
            break;
        case MODEL_3508:
            if (config->motor_set_id < 5) {
                motor_instances[config->bsp_can_index][0][config->motor_set_id - 1] = obj;  //若为3508电机，存放在motor_instances数组中第0行，四个位置对应电机id 1-4
            } else {
                motor_instances[config->bsp_can_index][1][config->motor_set_id - 5] = obj;  //若为3508电机，存放在motor_instances数组中第1行，四个位置对应电机id 1-4
            }
            BSP_CAN_AddFilter(obj->config.bsp_can_index, 0x200 + config->motor_set_id);
            break;
        case MODEL_6020:
            if (config->motor_set_id < 5) {
                motor_instances[config->bsp_can_index][1][config->motor_set_id - 1] = obj;  //若为6020电机，存放在motor_instances数组中第1行，四个位置对应电机id 1-4
            } else {
                motor_instances[config->bsp_can_index][2][config->motor_set_id - 5] = obj;  //若为6020电机，存放在motor_instances数组中第2行，四个位置对应电机id 1-4
            }
            BSP_CAN_AddFilter(obj->config.bsp_can_index, 0x204 + config->motor_set_id);
            break;
        default:
            break;
    }

    return obj;
}

void CanMotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len) {  //注册回调函数后，每当电机给主控回调信息，主控在can上收到后会调用此函数，并根据接收到的can线路调用Can_Motor_FeedbackData_Updat函数交给对应此can上的各个电机
    uint32_t model_3508_2006_id = identifier - 0x200;
    uint32_t model_6020_id = identifier - 0x204;
    // if (model_3508_2006_id > 8) return;
    if (model_3508_2006_id < 9 && motors_id[can_id][MODEL_2006][model_3508_2006_id - 1]) {
        Can_Motor_FeedbackData_Update(motor_instances[can_id][model_3508_2006_id < 5 ? 0 : 1][model_3508_2006_id < 5 ? model_3508_2006_id - 1 : model_3508_2006_id - 5], data);
    } else if (model_3508_2006_id < 9 && motors_id[can_id][MODEL_3508][model_3508_2006_id - 1]) {
        Can_Motor_FeedbackData_Update(motor_instances[can_id][model_3508_2006_id < 5 ? 0 : 1][model_3508_2006_id < 5 ? model_3508_2006_id - 1 : model_3508_2006_id - 5], data);
    } else if (model_6020_id < 9 && motors_id[can_id][MODEL_6020][model_6020_id - 1]) {
        Can_Motor_FeedbackData_Update(motor_instances[can_id][model_6020_id < 5 ? 1 : 2][model_6020_id < 5 ? model_6020_id - 1 : model_6020_id - 5], data);
    }
}

void Can_Motor_FeedbackData_Update(can_motor *obj, uint8_t *data) {  //电机数值更新函数,写入规则见对应电机的使用手册，对应变量的含义见can_motor.h的定义
    obj->last_fdbPosition = obj->fdbPosition;
    obj->fdbPosition = ((short)data[0]) << 8 | data[1];                                     //将新位置赋值给编码器位置反馈变量
    if (obj->last_fdbPosition == FDB_INIT_VALUE) obj->last_fdbPosition = obj->fdbPosition;  //若编码器位置反馈是初始值
    obj->fdbSpeed = ((short)data[2]) << 8 | data[3];                                        //更新电机反馈速度
    obj->electric_current = ((short)data[4]) << 8 | data[5];                                //更新电流
    if (obj->config.motor_model == MODEL_2006) {                                            //不同类型的电机温度写入规则不同
        obj->temperature = 0;
    } else {
        obj->temperature = data[6];
    }
    if (obj->fdbPosition - obj->last_fdbPosition > 4096)  //通过编码器计算圈数
        obj->round--;
    else if (obj->fdbPosition - obj->last_fdbPosition < -4096)
        obj->round++;
    obj->last_real_position_8192 = obj->real_position_8192;  //更新过零处理后的数据
    obj->real_position_8192 = obj->fdbPosition + obj->round * 8192;
    obj->real_position = obj->real_position_8192 * 360.0 / 8192.0;

    float position_delta = obj->real_position_8192 - obj->last_real_position_8192;  //计算位置变化量
    if (obj->position_queue->cq_len == VELOCITY_WINDOW) {
        float *now = circular_queue_pop(obj->position_queue);  //循环队列信息更新
        obj->position_sum -= *now;
    }
    circular_queue_push(obj->position_queue, &position_delta);
    obj->position_sum += position_delta;
    float vnow = obj->position_sum * 43.9453125 / obj->position_queue->cq_len;  //计算真实速度
    //obj->velocity = 0.2f * obj->velocity + 0.8f * vnow;
    obj->velocity =  obj->fdbSpeed*6;
    obj->monitor->reset(obj->monitor);  //重置监视器
}

void Can_Motor_Calc_Send() {  //主控发布控制信息函数
    for (int can_index = 0; can_index < 2; can_index++) {
        for (int identifier = 0; identifier < 3; identifier++) {  //循环不断寻找对应控制的电机
            uint8_t identifier_send = 0;
            short buf[4] = {0};
            for (int id = 0; id < 4; id++) {
                can_motor *obj = motor_instances[can_index][identifier][id];
                if (obj == NULL) continue;
                //电机掉线判断，如果同一个包内的所有的电机都掉线，那么这个包将不会发出
                if (is_Offline(obj->monitor)) continue;
                identifier_send = 1;
                if (obj->config.position_fdb_model == OTHER_FDB) {  //根据电机选取的不同反馈类型给pid所需的反馈量赋值
                    obj->motor_controller->fdb_position = *obj->config.position_fdb;
                } else {
                    obj->motor_controller->fdb_position = obj->real_position;
                }
                if (obj->config.speed_fdb_model == OTHER_FDB) {
                    obj->motor_controller->fdb_speed = *obj->config.speed_fdb;
                } else {
                    obj->motor_controller->fdb_speed = obj->velocity;
                }
                UNIVERSAL_Calc(obj->motor_controller);           //调用电机的pid配置文件计算pid输出
                buf[id] = (short)obj->motor_controller->output;  //将输出存入数组供后续发送使用
                if (obj->config.output_model == MOTOR_OUTPUT_REVERSE) buf[id] *= -1;
                if (obj->enable == MOTOR_STOP) {  //软件停转
                    buf[id] = 0;
                }
            }
            // 如果此标识符(identifier)对应的四个电机里至少有一个被注册，就发送这个标识符的报文，如果全部没有被注册，则这个标识符无需发送
            if (identifier_send) {
                Can_Motor_Send(can_index, identifiers[identifier], buf[0], buf[1], buf[2], buf[3]);
            }
        }
    }
}

void Can_Motor_Send(uint8_t can_id, uint16_t identifier, short motor_data_id1, short motor_data_id2, short motor_data_id3, short motor_data_id4) {  //输出转化数据并打包发送函数（一个包对应一条can上的所有电机需要的控制数据）
    static uint8_t data[8];
    data[0] = (uint8_t)(motor_data_id1 >> 8);
    data[1] = (uint8_t)(motor_data_id1 & 0x00FF);
    data[2] = (uint8_t)(motor_data_id2 >> 8);
    data[3] = (uint8_t)(motor_data_id2 & 0x00FF);
    data[4] = (uint8_t)(motor_data_id3 >> 8);
    data[5] = (uint8_t)(motor_data_id3 & 0x00FF);
    //对于标识符为0x2FF的情况，则data[6]和data[7]为NULL
    if (identifier == 0x200 || identifier == 0x1FF) {
        data[6] = (uint8_t)((motor_data_id4 & 0xFF00) >> 8);
        data[7] = (uint8_t)(motor_data_id4 & 0x00FF);
    }
    BSP_CAN_Send(can_id, identifier, data, 8);
}
