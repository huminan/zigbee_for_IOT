#ifndef MOTOR_H
#define MOTOR_H

#include "hal_timer.h"

#define MOTOR_STEP_1    P0_5
#define MOTOR_STEP_2    P0_7

#define MOTOR_DIR_1    P0_4
#define MOTOR_DIR_2    P0_6

#define MOTOR_MAX_OPERATION     5

#define MOTOR1_UPDATE_EVT        0x0020
#define MOTOR2_UPDATE_EVT        0x0021

void Timer3_uSec(void);

#endif