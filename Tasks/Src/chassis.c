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
PID_t chassis_thigh_vel_pid;
#endif
PID_t chassis_thigh_angle_pid;
void Chassis_Init()
{
    Toe_Init();
    Thigh_Init();
    PID_Init(&chassis_toe_turn_pid, 40.0f, 0.0f, 300.0f, 14000.0f, 14000.0f, 0.0f);
    PID_Init(&chassis_toe_angle_pid, 120.0f, 0.0f, 6000.0f, 14000.0f, 14000.0f, 0.0f);
    #ifdef NORMAL
    PID_Init(&chassis_toe_vel_pid, 1500.0f, .0f, 1000.0f, 14000.0f, 14000.f, 0.0f);
    #else
    PID_Init(&chassis_thigh_vel_pid, 0.01f, .0f, 0.0f, 30.0f, 30.f, 0.0f);
    #endif
    PID_Init(&chassis_thigh_angle_pid, .003f, 0.000000055f, .01f, 30.0f, 30.0f, 0.0f);
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

    /* Manipulation - Height */
    // chassis.target_height += g_remote.controller.right_stick.y / 6600000.0f;
    __MAX_LIMIT(g_chassis.target_height, 0.1f, 0.3f);

    /* Manipulation - Leg Angle */
    #ifdef MANUAL_LEG_ANG
    chassis.target_leg_ang += g_remote.controller.left_stick.x / 220000.0f;
    __MAX_LIMIT(chassis.target_leg_ang, (90.0f - THIGH_ANG_RANGE) * DEG_TO_RAD, (90.0f + THIGH_ANG_RANGE) * DEG_TO_RAD);
    #endif
    /* Manipulation - Forward Velocity */
    g_chassis.target_vel = remote.controller.left_stick.y / 660.0f;
    
    /* Manipulation - Yaw */
    g_chassis.target_yaw += (-remote.controller.right_stick.x / 3000.0f);

    /* Estimation */
    g_chassis.current_vel = (toe[0].velocity * TOE_LEFT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS) / 2.0f + 
                    (toe[1].velocity * TOE_RIGHT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS) / 2.0f;
    
    g_chassis.last_raw_yaw = g_chassis.current_raw_yaw;
    g_chassis.current_raw_yaw = g_imu.deg.yaw;
    g_chassis.current_yaw = g_chassis.turn_count * 360.0f + g_chassis.current_raw_yaw;
    if (fabs(g_chassis.current_raw_yaw - g_chassis.last_raw_yaw) > 1.9f * PI)
    {
        if ((g_chassis.current_raw_yaw - g_chassis.last_raw_yaw) < 0)
            g_chassis.turn_count++;
        else
            g_chassis.turn_count--;
    }
    
    g_chassis.current_pitch = g_imu.deg.pitch;
    /* State Machine */
    g_chassis.last_state = g_chassis.state;
    if ((g_chassis.target_vel != 0) | (g_chassis.target_yaw != 0)) 
        g_chassis.state = MOVING;
    else
        g_chassis.state = REST;

    if (g_chassis.state != g_chassis.last_state)
    {
        #ifdef NORMAL
        /* Reset the toe velocity pid, so that robot will update the new balancing location */
        PID_Reset(&chassis_toe_vel_pid);
        #endif
    }
}

void Chassis_ToeControl()
{
    #ifdef NORMAL
    float toe_vel_output = PID_Output(&chassis_toe_vel_pid, g_chassis.current_vel - g_chassis.target_vel);
    float toe_ang_output = -PID_Output(&chassis_toe_angle_pid, balance_ang - g_chassis.current_pitch);
    float toe_turn_output = PID_Output(&chassis_toe_turn_pid, g_chassis.target_yaw - g_chassis.current_yaw);
    float toe_left_target_vel = toe_ang_output + toe_vel_output - toe_turn_output;
    float toe_right_target_vel = toe_ang_output + toe_vel_output + toe_turn_output;
    __MAX_LIMIT(toe_left_target_vel, -14000, 14000);
    __MAX_LIMIT(toe_right_target_vel, -14000, 14000);
    if (g_remote.controller.left_switch == MID) 
        Toe_VelCtrl(toe_left_target_vel, toe_right_target_vel);
    else
    {
        Toe_TorqCtrl(0,0);
        PID_Reset(&chassis_toe_angle_pid);
        PID_Reset(&chassis_toe_vel_pid);
    }
    #else 
    float toe_ang_output = -PID_Output(&chassis_toe_angle_pid, balance_ang - imu_angle_deg[1]);
    float toe_target_vel = toe_ang_output;
    __MAX_LIMIT(toe_target_vel, -14000, 14000);
    if (g_remote.controller.left_switch == MID) 
        Toe_VelCtrl(toe_target_vel, toe_target_vel);
    else
    {
        Toe_TorqCtrl(0,0);
        PID_Reset(&chassis_toe_angle_pid);
    }
    #endif
}

void Chassis_ThighControl()
{
    if (g_remote.controller.right_switch == UP)
    {
        #ifndef NORMAL
        chassis.target_leg_ang = chassis.target_leg_ang * 0.95f + 0.05f * (PI/2 + PID_Output(&chassis_thigh_vel_pid, chassis.target_vel - chassis.current_vel));
        #endif
    }
    __MAX_LIMIT(g_chassis.target_leg_ang, (90.0f - THIGH_ANG_RANGE) * DEG_TO_RAD, (90.0f + THIGH_ANG_RANGE) * DEG_TO_RAD);
    Thigh_Ctrl(g_chassis.target_height, g_chassis.target_leg_ang);
}

void Chassis_Enable()
{
    Thigh_Enable();
    Toe_Enable();
}

void Chassis_Disable()
{
    Thigh_Disable();
    Toe_Disable();
}


void Chassis_Ctrl(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
    Chassis_Init();
    while (1)
    {
        Chassis_Process(g_remote);
        if (g_chassis.enabled) 
        {
            Chassis_Enable();
            Chassis_ThighControl();
            Chassis_ToeControl();
        }
        else 
            Chassis_Disable();
        
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
