#ifndef _APP_H
#define _APP_H

#include "bsp_log.h"
#include "BMI088.h"
#include "bsp_log.h"
#include "can_motor.h"
#include "pub_sub.h"
#include "robot_def.h"
#include "stdint.h"
#include "super_cap_wuli.h"

void APP_Layer_Init();
void APP_Layer_default_loop();
// APP层的函数，输出调试信息
void APP_Log_Loop();
// APP层的函数，机器人命令层中枢，在app.h中声明并直接在rtos.c中执行
void APP_Loop();

#endif