#include "chassis.h"

#define K11 (5.7735f)
#define K12 (5.8364f)
#define K13 (-12.5286f)
#define K14 (-1.8065f)
#define K15 (-0.0034f)
#define K16 (-0.0011f)
#define K21 (-0.0373f)
#define K22 (-0.0371f)
#define K23 (0.0748f)
#define K24 (0.0111f)
#define K25 (-6.3245f)
#define K26 (-2.0000f)

Chassis_t g_chassis;

PID_t g_left_leg_length_pid, g_right_leg_length_pid;

void Chassis_Init()
{
    Toe_Init();
    Leg_Init();

    PID_Init(&g_left_leg_length_pid, 3, 0, 0, 10, 0, 0);
    PID_Init(&g_right_leg_length_pid, 3, 0, 0, 10, 0, 0);
    
}
void Chassis_Process(Remote_t remote)
{
    
    /* Manipulation - Enabled */
    if (g_remote.controller.right_switch == MID) g_chassis.enabled = 1;
    else if (g_remote.controller.right_switch == DOWN) g_chassis.enabled = 0;
    
    Leg_ForwardKinematics(&g_chassis.left_leg, leg_motors[3].pos - LEFT_REAR_OFFSET, leg_motors[0].pos - LEFT_FRONT_OFFSET, leg_motors[3].vel, leg_motors[0].vel);
    Leg_ForwardKinematics(&g_chassis.right_leg, leg_motors[1].pos - RIGHT_FRONT_OFFSET, leg_motors[2].pos - RIGHT_REAR_OFFSET, leg_motors[1].vel, leg_motors[2].vel);

    /* Estimation */
    g_chassis.current_disp = (toe[0].total_angle * TOE_LEFT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS) / 2.0f + 
                    (toe[1].total_angle * TOE_RIGHT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS) / 2.0f;
    g_chassis.current_vel = (toe[0].velocity * TOE_LEFT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS) / 2.0f + 
                    (toe[1].velocity * TOE_RIGHT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS) / 2.0f;
    g_chassis.current_pitch = g_imu.rad.pitch;
    g_chassis.current_pitch_dot = g_imu.bmi088_raw.gyro[1];
    g_chassis.target_height = INIT_CHASSIS_HEIGHT;

    g_chassis.last_theta = g_chassis.current_theta;
    g_chassis.current_theta = g_chassis.current_pitch + (g_chassis.left_leg.phi0 - g_chassis.right_leg.phi0) / 2.0f;
    g_chassis.current_theta_dot = (g_chassis.current_theta - g_chassis.last_theta) / 0.004f;

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

void Chassis_LegControl(float height, Leg_t *left, Leg_t *right, PID_t *left_leg_length_pid, PID_t *right_leg_length_pid)
{
    Leg_VMC(left, PID_Output(left_leg_length_pid, height - left->length) + ROBOT_WEIGHT / 2, 0);
    Leg_VMC(right, PID_Output(right_leg_length_pid, height - left->length) + ROBOT_WEIGHT / 2, 0);
    Leg_CtrlTorq(left->torq4, right->torq1, right->torq4, left->torq1);
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
            Toe_TorqCtrl(0, 0);
            //Chassis_LegControl(g_chassis.target_height, &g_chassis.left_leg, &g_chassis.right_leg, &g_left_leg_length_pid, &g_right_leg_length_pid);
            
        }
        else 
            Chassis_Disable();
        
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
