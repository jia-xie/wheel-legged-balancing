#ifndef THIGH_H
#define THIGH_H

#include "robot_param.h"
#include "DM4310.h"
#include "can.h"
#include "math.h"
#include "pid.h"
#include "user_math.h"

extern DM4310_Info_t thigh[4];

extern void Thigh_Init(void);
extern void Thigh_Enable(void);
extern void Thigh_Disable(void);
extern void Thigh_Ctrl(float height, float angle);
extern void Thigh_CtrlLeg(float left_front, float right_front, float right_rear, float left_rear);
extern void Thigh_InverseKinematics(float height, float leg_angle, float *thigh_1, float *thigh_2);
#endif
