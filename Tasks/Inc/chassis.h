#ifndef CHASSIS_H
#define CHASSIS_H
#include "FreeRTOS.h"

#include "math.h"
/* Devices */
#include "remote.h"
#include "DM4310.h"
#include "leg.h"
#include "toe.h"
#include "robot_param.h"
#include <stdlib.h>
#include "pid.h"
#include "imu_task.h"

enum State
{
    REST = 0,
    MOVING
};

typedef struct
{
    uint8_t enabled;
    enum State last_state, state;

    float target_vel;
    float target_height;

    float current_disp;
    float current_vel;
    float current_theta;
    float current_theta_dot;
    float last_theta;
    float current_pitch;
    float current_pitch_dot;

    Leg_t left_leg;
    Leg_t right_leg;
    float current_height;
} Chassis_t;

extern float speed;
extern Chassis_t g_chassis;
extern float kp;
extern float kd;
extern float balance_ang;
#endif
