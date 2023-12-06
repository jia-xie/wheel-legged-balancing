#ifndef THIGH_H
#define THIGH_H

#include "robot_param.h"
#include "DM4310.h"
#include "can.h"
#include "math.h"
#include "pid.h"
#include "user_math.h"

typedef struct
{
    float phi1;
    float phi4;
} Leg_Info_t;

typedef struct leg
{
    float phi1;
    float phi2;
    float phi3;
    float phi4;
    
    float phi1_dot;
    float phi2_dot;
    float phi3_dot;
    float phi4_dot;

    float phi0;
    float phi0_dot;
    float length;
    float length_dot;
} Leg_t;


extern DM4310_Info_t leg_motors[4];

extern void Leg_Init(void);
extern void Leg_Enable(void);
extern void Leg_Disable(void);
extern void Leg_ForwardKinematics(Leg_t *leg, float phi1, float phi2, float phi1_dot, float phi2_dot);
extern void Leg_Ctrl(float height, float angle);
extern void Leg_CtrlLeg(float left_front, float right_front, float right_rear, float left_rear);
extern void Leg_InverseKinematics(float height, float leg_angle, float *leg_1, float *leg_2);
#endif
