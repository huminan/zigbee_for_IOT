#ifndef MOTOR_H
#define MOTOR_H

#include "hal_timer.h"

#define MOTOR_STEP_1    P1_3
#define MOTOR_STEP_2    P1_5
#define MOTOR_STEP_3    P1_7

#define MOTOR_DIR_1     P1_2
#define MOTOR_DIR_2     P1_4
#define MOTOR_DIR_3     P1_6

#define MOTOR_MAX_OPERATION     5

#define MOTOR_UPDATE_EVT        0x5000

void Timer3_uSec(void);

#endif