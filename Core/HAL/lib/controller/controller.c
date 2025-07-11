#include <controller.h>
#include <stdlib.h>
#include <string.h>
float fal(float e, float a, float delta);
void UNIVERSAL_Calc(controller* obj) {
    if (obj->calc_type == CLASSIC) {  //经典一阶控制
        obj->rank1.fdb = obj->fdb_speed;
        obj->rank1.ref = obj->ref_speed;
        CLASSIC_PID_Calc(&obj->rank1);
        obj->output = obj->rank1.output;
    } else if (obj->calc_type == CASCADE) {  //串级控制
        obj->rank2.fdb = obj->fdb_speed;
        obj->rank1.ref = obj->ref_position;
        obj->rank1.fdb = obj->fdb_position;
        CLASSIC_PID_Calc(&obj->rank1);
        obj->rank2.ref = obj->rank1.output;
        CLASSIC_PID_Calc(&obj->rank2);
        obj->output = obj->rank2.output;
    } else if (obj->calc_type == STATE_SPACE) {  //二阶状态空间控制器 应用lqr时应该外部计算K1，K2以及反馈单位化系数c1，c2，以及单位化输出转换系数b0，b1
        obj->state_space_controller.target = obj->ref_position;
        obj->state_space_controller.y1 = obj->fdb_position;
        obj->state_space_controller.y2 = obj->fdb_speed;
        obj->state_space_controller.output = 0;
        //误差
        obj->state_space_controller.z1 = obj->state_space_controller.config.c1 * (obj->state_space_controller.y1 - obj->state_space_controller.target);
        obj->state_space_controller.z2 = obj->state_space_controller.config.c2 * obj->state_space_controller.y2;
        obj->state_space_controller.output = -obj->state_space_controller.config.K1 * obj->state_space_controller.z1 - obj->state_space_controller.config.K2 * obj->state_space_controller.z2;
        //状态量积分补偿
        if (obj->state_space_controller.integrator.error_sum > obj->state_space_controller.integrator.config.error_max) {
            obj->state_space_controller.integrator.error_sum = obj->state_space_controller.integrator.config.error_max;
        }
        if (obj->state_space_controller.integrator.error_sum < -obj->state_space_controller.integrator.config.error_max) {
            obj->state_space_controller.integrator.error_sum = -obj->state_space_controller.integrator.config.error_max;
        }
        if (fabs(obj->state_space_controller.z1) <= obj->state_space_controller.integrator.config.range) {
            obj->state_space_controller.integrator.error_sum += obj->state_space_controller.z1 * obj->state_space_controller.config.h;  //积分量
            obj->state_space_controller.output -= obj->state_space_controller.integrator.error_sum * obj->state_space_controller.integrator.config.KI;
        } else {
            obj->state_space_controller.integrator.error_sum = fsgn(obj->state_space_controller.z1) * obj->state_space_controller.integrator.config.error_preload;
        }
        //单位化输出量转化为输出数据
        obj->output = obj->state_space_controller.config.b0 + obj->state_space_controller.config.b1 * obj->state_space_controller.output;
        obj->output += obj->state_space_controller.config.feedback_forward;
        if (obj->output > obj->state_space_controller.config.outputmax) {
            obj->output = obj->state_space_controller.config.outputmax;
        }
        if (obj->output < -obj->state_space_controller.config.outputmax) {
            obj->output = -obj->state_space_controller.config.outputmax;
        }
    }
    if (obj->leso.config.SWITCH == MYENABLE) {  //线性扩张状态观测器
        obj->leso.u = *(obj->leso.config.p_input);
        float z_expand;
        z_expand = obj->leso.z_expand;
        obj->leso.x1 = obj->fdb_speed;
        obj->leso.z_expand = z_expand + obj->leso.config.h * obj->leso.config.l_expand * (obj->leso.x1 - obj->leso.z1);
        if (obj->leso.z_expand > obj->leso.config.z_expand_max) {
            obj->leso.z_expand = obj->leso.config.z_expand_max;
        } else if (obj->leso.z_expand < -obj->leso.config.z_expand_max) {
            obj->leso.z_expand = -obj->leso.config.z_expand_max;
        }
        obj->leso.z1 = obj->leso.z1 + obj->leso.config.h * (z_expand + obj->leso.config.l1 * (obj->leso.x1 - obj->leso.z1) + obj->leso.config.b * obj->leso.u);
        obj->output -= 1 * (obj->leso.z_expand) / obj->leso.config.b;
    }
    obj->z = obj->leso.z1;
}

void CLASSIC_PID_Calc(classic_controller* pid) {
    float u = 0;
    pid->error[2] = pid->error[1];        //上上次误差
    pid->error[1] = pid->error[0];        //上次误差
    pid->error[0] = pid->ref - pid->fdb;  //本次误差
    if (pid->config.NLSEF_SWITCH == MYENABLE) {
        if (pid->config.alpha == 0.5) {
            u = fsgn(pid->error[0]) * sqrt(fabs(pid->error[0]));
        } else {
            u = fsgn(pid->error[0]) * fabs(fal(fabs(pid->error[0]), pid->config.alpha, pid->config.delta));
        }
    }
    if (pid->config.PID_Mode == PID_POSITION)  //位置式PID
    {
        if (fabs(pid->error[0]) <= pid->integrator.config.range) {  //在积分范围
            pid->integrator.error_sum += pid->error[0];             //积分上限判断
            if (pid->integrator.error_sum > pid->integrator.config.error_max) pid->integrator.error_sum = pid->integrator.config.error_max;
            if (pid->integrator.error_sum < -pid->integrator.config.error_max) pid->integrator.error_sum = -pid->integrator.config.error_max;
            pid->output = pid->config.KP_NL * u + pid->config.KP * pid->error[0] + pid->integrator.config.KI * pid->integrator.error_sum + pid->config.KD * (pid->error[0] - pid->error[1]);
        } else {  //不在积分范围
            pid->integrator.error_sum = fsgn(pid->error[0]) * pid->integrator.config.error_preload;
            pid->output = pid->config.KP_NL * u + pid->config.KP * pid->error[0] + pid->config.KD * (pid->error[0] - pid->error[1]);
        }

    } else if (pid->config.PID_Mode == PID_DELTA)  //增量式PID
    {
        pid->output += pid->config.KP * (pid->error[0] - pid->error[1]) + pid->config.KD * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]) + pid->integrator.config.KI * pid->error[0];
    }
    pid->output_unlimited = pid->output;
    //输出限幅
    if (pid->output > pid->config.outputmax) {
        pid->output = pid->config.outputmax;
    }
    if (pid->output < -pid->config.outputmax) {
        pid->output = -pid->config.outputmax;
    }
}

//经典pid配置
void CLASSIC_PID_Setconfig(controller_config* obj, float kp, float kd, float ki, float error_max, float outputmax, float error_preload, float irange) {
    obj->calc_type = CLASSIC;
    obj->leso_config.SWITCH = MYDISABLE;
    obj->rank1_config.PID_Mode = PID_POSITION;
    obj->rank1_config.D_ahead = MYDISABLE;
    obj->rank1_config.integrator_config.error_max = error_max;
    obj->rank1_config.NLSEF_SWITCH = MYDISABLE;
    obj->rank1_config.integrator_config.error_preload = error_preload;
    obj->rank1_config.integrator_config.KI = ki;
    obj->rank1_config.integrator_config.error_max = error_max;
    obj->rank1_config.integrator_config.range = irange;
    obj->rank1_config.KD = kd;
    obj->rank1_config.KP = kp;
    obj->rank1_config.outputmax = outputmax;
}

//串级pid配置
void CASCADE_PID_Setconfig(controller_config* obj, float kp_pos, float kd_pos, float ki_pos, float error_max_pos, float outputmax_pos, float error_preload_pos, float irange_pos, float kp_spd, float kd_spd, float ki_spd, float error_max_spd, float outputmax_spd, float error_preload_spd, float irange_spd) {
    obj->calc_type = CASCADE;
    obj->leso_config.SWITCH = MYDISABLE;
    obj->rank1_config.PID_Mode = PID_POSITION;
    obj->rank1_config.D_ahead = MYDISABLE;
    obj->rank1_config.integrator_config.error_max = error_max_pos;
    obj->rank1_config.NLSEF_SWITCH = MYDISABLE;
    obj->rank1_config.integrator_config.error_preload = error_preload_pos;
    obj->rank1_config.integrator_config.KI = ki_pos;
    obj->rank1_config.integrator_config.error_max = error_max_pos;
    obj->rank1_config.integrator_config.range = irange_pos;
    obj->rank1_config.KD = kd_pos;
    obj->rank1_config.KP = kp_pos;
    obj->rank1_config.outputmax = outputmax_pos;
    obj->rank2_config.PID_Mode = PID_POSITION;
    obj->rank2_config.D_ahead = MYDISABLE;
    obj->rank2_config.integrator_config.error_max = error_max_spd;
    obj->rank2_config.NLSEF_SWITCH = MYDISABLE;
    obj->rank2_config.integrator_config.error_preload = error_preload_spd;
    obj->rank2_config.integrator_config.KI = ki_spd;
    obj->rank2_config.integrator_config.error_max = error_max_spd;
    obj->rank2_config.integrator_config.range = irange_spd;
    obj->rank2_config.KD = kd_spd;
    obj->rank2_config.KP = kp_spd;
    obj->rank2_config.outputmax = outputmax_spd;
}

//一阶滑膜配置
void SMC_Setconfig(controller_config* obj, float kp, float kd, float ki, float error_max, float outputmax, float error_preload, float irange, float kp_nl) {
    obj->calc_type = CLASSIC;
    obj->leso_config.SWITCH = MYDISABLE;
    obj->rank1_config.PID_Mode = PID_POSITION;
    obj->rank1_config.D_ahead = MYDISABLE;
    obj->rank1_config.integrator_config.error_max = error_max;
    obj->rank1_config.NLSEF_SWITCH = MYENABLE;
    obj->rank1_config.integrator_config.error_preload = error_preload;
    obj->rank1_config.integrator_config.KI = ki;
    obj->rank1_config.integrator_config.error_max = error_max;
    obj->rank1_config.integrator_config.range = irange;
    obj->rank1_config.KD = kd;
    obj->rank1_config.KP = kp;
    obj->rank1_config.outputmax = outputmax;
    obj->rank1_config.KP_NL = kp_nl;
    obj->rank1_config.alpha = 0.5;
    obj->rank1_config.delta = 0;
}
//一阶ladrc配置
void LADRC_Setconfig(controller_config* obj, float kp, float kd, float ki, float error_max, float outputmax, float error_preload, float irange) {
    obj->calc_type = CLASSIC;
    obj->leso_config.SWITCH = MYDISABLE;
    obj->rank1_config.PID_Mode = PID_POSITION;
    obj->rank1_config.D_ahead = MYDISABLE;
    obj->rank1_config.integrator_config.error_max = error_max;
    obj->rank1_config.NLSEF_SWITCH = MYDISABLE;
    obj->rank1_config.integrator_config.error_preload = error_preload;
    obj->rank1_config.integrator_config.KI = ki;
    obj->rank1_config.integrator_config.error_max = error_max;
    obj->rank1_config.integrator_config.range = irange;
    obj->rank1_config.KD = kd;
    obj->rank1_config.KP = kp;
    obj->rank1_config.outputmax = outputmax;
    // obj->leso_config.b = b;
    // obj->leso_config.h = h;
    // obj->leso_config.l1 = l;
    // obj->leso_config.c1 = c;
    // obj->leso_config.l_expand = l_expand;
    // obj->leso_config.z_expand_max = z_expand_max;
}
//二阶lqr设置
void LQR_Setconfig(controller_config* obj, float K1, float K2, float b0, float b1, float ki, float k_pos_fdb, float outputmax, float k_spd_fdb, float irange, float error_preload) {
    obj->calc_type = STATE_SPACE;
    obj->leso_config.SWITCH = MYDISABLE;
    obj->state_space_controller_config.h = 0.001;
    obj->state_space_controller_config.outputmax = outputmax;
    obj->state_space_controller_config.integrator_config.KI = ki;
    obj->state_space_controller_config.integrator_config.error_preload = error_preload;
    obj->state_space_controller_config.integrator_config.range = irange;
    obj->state_space_controller_config.b0 = b0;
    obj->state_space_controller_config.b1 = b1;
    obj->state_space_controller_config.c1 = k_pos_fdb;
    obj->state_space_controller_config.c2 = k_spd_fdb;
    obj->state_space_controller_config.K1 = K1;
    obj->state_space_controller_config.K2 = K2;
}

//因为通用控制器内容较多，需要按情况分配内存
controller* create_controller(controller_config* config) {
    controller* obj = malloc(sizeof(controller));
    memset(obj, 0, sizeof(controller));
    obj->calc_type = config->calc_type;
    // obj->config = *config;
    if (config->leso_config.SWITCH == MYENABLE) {
        obj->leso.config = config->leso_config;
    }
    if (config->calc_type == CLASSIC) {
        obj->rank1.config = config->rank1_config;
        obj->rank1.integrator.config = obj->rank1.config.integrator_config;
    } else if (config->calc_type == CASCADE) {
        obj->rank1.config = config->rank1_config;
        obj->rank1.integrator.config = obj->rank1.config.integrator_config;
        obj->rank2.config = config->rank2_config;
        obj->rank2.integrator.config = obj->rank2.config.integrator_config;
    } else if (config->calc_type == STATE_SPACE) {
        obj->state_space_controller.config = config->state_space_controller_config;
        obj->state_space_controller.integrator.config = obj->state_space_controller.config.integrator_config;
    }
    return obj;
}