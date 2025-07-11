#ifndef _CONTROLLER_H
#define _CONTROLLER_H
#include <pid.h>

#include "common.h"
#include "math.h"
#define MYENABLE 1
#define MYDISABLE 0U
enum calc_type_e { CLASSIC = 0,
                   CASCADE,
                   STATE_SPACE };

#pragma pack(1)

//线性扩张状态观测器
typedef struct LESO_config_t {
    uint8_t SWITCH;
    float l_expand;
    float l1;  //观测器闭环系数
    float b;   //输入比例增益
    float h;   //步长
    float z_expand_max;
    short* p_input;
} leso_config;

typedef struct LESO_t {
    leso_config config;
    float z1;
    float x1;
    float u;  //对系统的输入
    float z_expand;
} leso;
//积分补偿器
typedef struct INTEGRATOR_config_t {
    float KI;
    float range;
    float error_max;
    float error_preload;
} integrator_config;
//积分补偿器
typedef struct INTEGRATOR_t {
    integrator_config config;
    float error_sum;
} integrator;
//状态空间控制器
typedef struct STATE_SPACE_CONTROLLER_config_t {
    uint8_t rank;
    float K1, K2;
    float c1, c2;  //反馈值变换系数
    float b0, b1;
    integrator_config integrator_config;
    float feedback_forward;
    float outputmax;
    float h;
} state_space_controller_config;
typedef struct STATE_SPACE_CONTROLLER_t {
    state_space_controller_config config;
    integrator integrator;
    float target;  //目标值
    float y1, y2;  //原始反馈值
    float z1, z2;  //状态变量
    float output;
    float output_unlimited;  // 经outputMax限制前的原始输出
} state_space_controller;
//传统控制器
typedef struct CLASSIC_CONTROLLER_config_t {
    uint8_t NLSEF_SWITCH;
    float alpha;
    float delta;
    float KP_NL;
    float KD;
    float KP;
    integrator_config integrator_config;
    enum PID_Mode_e PID_Mode;
    uint8_t D_ahead;
    float feedback_forward[2];
    float outputmax;
} classic_controller_config;
typedef struct CLASSIC_CONTROLLER_t {
    integrator integrator;
    classic_controller_config config;
    float error[3];
    float fdb;
    float ref;
    float output;
    float output_unlimited;  // 经outputMax限制前的原始输出
} classic_controller;

//通用自定义控制器
typedef struct CONTROLLER_config_t {
    classic_controller_config rank1_config;                       //一阶参数
    classic_controller_config rank2_config;                       //二阶参数
    state_space_controller_config state_space_controller_config;  //状体空间参数
    enum calc_type_e calc_type;                                   //输出的计算方式（经典一阶，串级，状态空间
    leso_config leso_config;                                      //线性扩张状态观测器
} controller_config;

typedef struct CONTROLLER_t {
    enum calc_type_e calc_type;
    classic_controller rank1;  //一阶控制器
    classic_controller rank2;  //二阶控制器
    state_space_controller state_space_controller;
    leso leso;    //扩张状态观测器
    float output;  //整体输出
    float ref_speed;
    float ref_position;
    float fdb_speed;
    float fdb_position;
    float z;
} controller;

#pragma pack()
void SMC_Setconfig(controller_config* obj, float kp, float kd, float ki, float error_max, float outputmax, float error_preload, float irange, float kp_nl);
controller* create_universal_controller(controller_config* config);
void CASCADE_PID_Setconfig(controller_config* obj, float kp_pos, float kd_pos, float ki_pos, float error_max_pos, float outputmax_pos, float error_preload_pos, float irange_pos, float kp_spd, float kd_spd, float ki_spd, float error_max_spd, float outputmax_spd, float error_preload_spd, float irange_spd);
controller* Create_controller(controller_config* config);
void UNIVERSAL_Calc(controller* obj);
void CLASSIC_PID_Calc(classic_controller* pid);
void LQR_Setconfig(controller_config* obj, float K1, float K2, float b0, float b1, float ki, float k_pos_fdb, float outputmax, float k_spd_fdb, float irange, float error_preload);
void CLASSIC_PID_Setconfig(controller_config* obj, float kp, float kd, float ki, float error_max, float outputmax, float error_preload, float irange);
#endif