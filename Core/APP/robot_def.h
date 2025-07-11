#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_

// 定义主控类型 方便统一板间can通信写法
// 按照要烧录的主控类型 **必须**定义且仅定义一个 另一个注释
#define GIMBAL_BOARD
// #define CHASSIS_BOARD

#include "stdint.h"
#include "stdlib.h"
// 部分外设数据定义
#include "imu_data.h"
#include "referee_def.h"

// 机器人结构参数定义
#include "robot_struct.h"

// 各模块pub_sub的参数结构体
// 各部分对外接口统一存放
// 各部分通过pub_sub方式“沟通”的“通讯协议”

#pragma pack(1)
/** 机器人模式定义 **/
// 机器人总模式
typedef enum Robot_mode_e { robot_stop = 0, robot_run } Robot_mode;

/** 机器人模块控制量定义 **/
typedef enum Chassis_mode_e {
    chassis_stop = 0,
    chassis_run,
    // chassis_vision_control_run
} Chassis_mode;
typedef struct Cmd_chassis_t {
    Chassis_mode mode;
    // 写需要的控制量
    float vx;
    float wz;
} Cmd_chassis;
// 对模块的控制量
typedef struct Cmd_module_t {
    Robot_mode mode;
    // others
} Cmd_module;
// 模块回传cmd的数据
typedef struct Upload_chassis_t {
    // 写需要的控制量
    float vx;
    float wz;
} Upload_chassis;

#pragma pack()
#endif