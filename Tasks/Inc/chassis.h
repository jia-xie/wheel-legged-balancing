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
    float target_yaw;
    float target_leg_ang;
    float target_leg_1;
    float target_leg_2;

    float current_vel;
    float current_turning;
    float current_height;
    float current_pitch;
    float current_raw_yaw;
    float current_yaw;
    float current_leg_ang;
    float current_leg_1;
    float current_leg_2;
    
    float last_raw_yaw;
    int16_t turn_count;
} Chassis_t;

extern float speed;
extern Chassis_t g_chassis;
extern float kp;
extern float kd;
extern float balance_ang;
#endif
