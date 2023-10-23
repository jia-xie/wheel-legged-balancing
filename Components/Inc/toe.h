#ifndef TOE_H
#define TOE_H

#include "robot_param.h"
#include "mf_motor.h"
#include "pid.h"
#include "user_math.h"

#define TOE_LEFT_DIR 1
#define TOE_RIGHT_DIR -1

extern void Toe_Enable(void);
extern void Toe_Disable(void);


extern MF_MOTOR_INFO_t toe[2];

extern void Toe_Init(void);
extern void Toe_VelCtrl(int32_t left, int32_t right);
extern void Toe_TorqCtrl(int32_t left, int32_t right);

#endif
