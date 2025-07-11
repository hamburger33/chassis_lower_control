#include "app.h"
#include "robot_def.h"
#include "BMI088.h"
#include "bsp_def.h"

#include <arm_math.h>
#include <math.h>

#include "bsp.h"
#include "bsp_random.h"
#include "common.h"
// 功能模块
#include "chassis.h"
// 控制模块
// #include "robot_cmd.h"

Chassis* chassis;
void APP_Layer_Init() {
    // robot_cmd = Robot_CMD_Create();
    chassis = Chassis_Create();
}

void APP_Loop() {
    // Robot_CMD_Update(robot_cmd);
    Chassis_Update(chassis);
}

// 打印输出等到ozone的窗口 用于测试项目
void APP_Log_Loop() {
    if (chassis->cmd_data != NULL) {
        printf_log("Mode: %d, VX: %f, WZ: %f\n", chassis->cmd_data->mode, chassis->cmd_data->vx, chassis->cmd_data->wz);
    } else {
        printf_log("No valid cmd_data available.\n");
    }
}

void APP_Layer_default_loop() {
    // if (chassis->imu->bias_init_success) {
    //     Buzzer_Update(internal_buzzer);
    // }
}
