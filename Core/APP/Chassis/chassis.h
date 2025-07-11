#ifndef _CHASSIS_H
#define _CHASSIS_H
/*
 * 本文件中，无论参照系，所有vx正方向为右，y正方向为前，旋转速度正方向为顺时针
 *
 *                 |  y正方向（底盘正前）
 *                 |
 *                 |
 *------------------------------------- x正方向（右）
 *                 |
 *                 |
 *                 |
 *                 |
 *
 */

#define CHASSIS_CMD_TOPIC      "chassis_cmd"
#define CHASSIS_STATUS_TOPIC   "chassis_status"
#define UART_INDEX        1          // 使用 huart2
#define CHASSIS_CMD_UART_ID     0x200      // 上位机命令帧 ID

#include "BMI088.h"
#include "bsp_log.h"
#include "can_motor.h"
#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
#include "super_cap_wuli.h"
#pragma pack(1)
typedef struct Chassis_t {
    //外设
    Super_cap_wuli *super_cap;
    can_motor *lf;
    can_motor *rf;
    can_motor *lb;
    can_motor *rb;  // forward back left right


    //底盘机构控制命令、回传数据
    Cmd_chassis *cmd_data;       // 接收到的指令数据
    Upload_chassis chassis_upload_data;  // 回传的数据
    Subscriber *chassis_cmd_suber;
    Publisher *chassis_status_pub;

    //底盘机构所需参数
    float offset_x;  // 旋转中心距离底盘的距离，云台位于正中心时默认设为0
    float offset_y;
    float proc_target_vx; //过程量
    float proc_target_vy; //过程量
    float proc_v_base; //过程量
    
} Chassis;
#pragma pack()
Chassis *Chassis_Create(void);
void Chassis_Update(Chassis *obj);
#endif
