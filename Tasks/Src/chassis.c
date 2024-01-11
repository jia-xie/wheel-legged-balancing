#include "chassis.h"

#define NORMAL
float a, b, c, d;
Chassis_t g_chassis;
float speed = 0;
float balance_ang = 0.0f;
PID_t chassis_toe_turn_pid;
PID_t chassis_toe_angle_pid;
#ifdef NORMAL
PID_t chassis_toe_vel_pid;
#else
PID_t chassis_leg_vel_pid;
#endif
PID_t chassis_leg_angle_pid;
void Chassis_Init()
{
    Toe_Init();
    Leg_Init();
    PID_Init(&chassis_toe_turn_pid, 40.0f, 0.0f, 300.0f, 14000.0f, 14000.0f, 0.0f);
    PID_Init(&chassis_toe_angle_pid, 120.0f, 0.0f, 3000.0f, 14000.0f, 14000.0f, 0.0f);
    #ifdef NORMAL
    PID_Init(&chassis_toe_vel_pid, 1500.0f, .0f, 400.0f, 14000.0f, 14000.f, 0.0f);
    #else
    PID_Init(&chassis_leg_vel_pid, 0.01f, .0f, 0.0f, 30.0f, 30.f, 0.0f);
    #endif
    PID_Init(&chassis_leg_angle_pid, .003f, 0.000000055f, .01f, 30.0f, 30.0f, 0.0f);
    g_chassis.turn_count = 0;
    
    g_chassis.last_raw_yaw = g_imu.deg.yaw;
    g_chassis.current_raw_yaw = g_imu.deg.yaw;
    g_chassis.target_yaw = g_chassis.current_raw_yaw;
    
    g_chassis.current_pitch = g_imu.deg.pitch;
    
    g_chassis.target_height = INIT_CHASSIS_HEIGHT;
    g_chassis.target_leg_ang = INIT_CHASSIS_ANGLE / 180.0f * PI;
}
void Chassis_Process(Remote_t remote)
{
    
    /* Manipulation - Enabled */
    if (g_remote.controller.right_switch == MID) g_chassis.enabled = 1;
    else if (g_remote.controller.right_switch == DOWN) g_chassis.enabled = 0;
    
    Leg_ForwardKinematics(&g_chassis.left_leg, leg_motors[3].pos - LEFT_REAR_OFFSET, leg_motors[0].pos - LEFT_FRONT_OFFSET, leg_motors[3].vel, leg_motors[0].vel);
    Leg_VMC(&g_chassis.left_leg, remote.controller.right_stick.y / 660.0f * 10, remote.controller.right_stick.x / 660.0f * 10);
}

void Chassis_Enable()
{
    Leg_Enable();
    Toe_Enable();
}

void Chassis_Disable()
{
    Leg_Disable();
    Toe_Disable();
}


void Chassis_Ctrl(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(4);
    Chassis_Init();
    while (1)
    {
        Chassis_Process(g_remote);
        if (g_chassis.enabled) 
        {
            Chassis_Enable();
            Leg_CtrlTorq(g_chassis.left_leg.torq4, 0, 0, g_chassis.left_leg.torq1);
        }
        else 
            Chassis_Disable();
        
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
