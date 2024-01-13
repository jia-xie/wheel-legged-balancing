#include "chassis.h"

#define K11 (1.9922f)
#define K12 (2.7070f)
#define K13 (-12.5286f)
#define K14 (-0.7812f)
#define K15 (-1.6701f)
#define K16 (-0.2316f)
#define K21 (-0.1765f)
#define K22 (-0.2402f)
#define K23 (1.0978f)
#define K24 (0.0975f)
#define K25 (-2.3353f)
#define K26 (-0.1242f)
              
Chassis_t g_chassis;
uint8_t enabled_count = 0;
PID_t chassis_toe_angle_pid;
PID_t chassis_toe_vel_pid;
PID_t chassis_left_leg_angle;
PID_t chassis_right_leg_angle;
float balance_ang = 0;

void Chassis_Init()
{
    Toe_Init();
    Leg_Init();

    PID_Init(&g_chassis.left_leg_length_pid, 200, 6, 900, 30, 0, 0);
    PID_Init(&g_chassis.right_leg_length_pid, 200, 6, 900, 30, 0, 0);

    PID_Init(&chassis_toe_vel_pid, 1500.0f, .0f, 1000.0f, 14000.0f, 14000.f, 0.0f);
    PID_Init(&chassis_toe_angle_pid, 120.0f, 0.0f, 6000.0f, 14000.0f, 14000.0f, 0.0f);

    PID_Init(&chassis_left_leg_angle, 6.0f, 0.0f, 50.0f, 2.0f, 2.0f, 0.0f);
    PID_Init(&chassis_right_leg_angle, 6.0f, 0.0f, 50.0f, 2.0f, 2.0f, 0.0f);

}

void Chassis_Process(Remote_t *remote)
{
    
    /* Manipulation - Enabled */
    if (remote->controller.right_switch == MID) g_chassis.enabled = 1;
    else if (remote->controller.right_switch == DOWN) g_chassis.enabled = 0;
    
    Leg_ForwardKinematics(&g_chassis.left_leg, leg_motors[3].pos - LEFT_REAR_OFFSET, leg_motors[0].pos - LEFT_FRONT_OFFSET, leg_motors[3].vel, leg_motors[0].vel);
    Leg_ForwardKinematics(&g_chassis.right_leg, leg_motors[1].pos - RIGHT_FRONT_OFFSET, leg_motors[2].pos - RIGHT_REAR_OFFSET, leg_motors[1].vel, leg_motors[2].vel);

    /* Estimation */
    g_chassis.left_leg.current_disp = (g_toe[0].total_angle * TOE_LEFT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS);
    g_chassis.right_leg.current_disp = -(g_toe[1].total_angle * TOE_RIGHT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS);
    g_chassis.left_leg.current_vel = (g_toe[0].velocity * TOE_LEFT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS);
    g_chassis.right_leg.current_vel = -(g_toe[1].velocity * TOE_RIGHT_DIR * DEG_TO_RAD * TOE_WHEEL_RADIUS);
    
    g_chassis.current_pitch = g_imu.rad.pitch;
    g_chassis.current_pitch_dot = g_imu.bmi088_raw.gyro[1];
    g_chassis.target_height = INIT_CHASSIS_HEIGHT + remote->controller.right_stick.y / 660.0f * 0.1f;

    g_chassis.left_leg.last_theta = g_chassis.left_leg.current_theta;
    g_chassis.left_leg.current_theta = g_chassis.current_pitch + g_chassis.left_leg.phi0 - PI / 2.0f;
    g_chassis.left_leg.current_theta_dot = (g_chassis.left_leg.current_theta - g_chassis.left_leg.last_theta) / 0.004f;

    g_chassis.right_leg.last_theta = g_chassis.right_leg.current_theta;
    g_chassis.right_leg.current_theta = g_chassis.current_pitch - (g_chassis.right_leg.phi0 - PI/2.0f);
    g_chassis.right_leg.current_theta_dot = (g_chassis.right_leg.current_theta - g_chassis.right_leg.last_theta) / 0.004f;
}
void Chassis_ToeControl()
{
    float toe_vel_output = PID_Output(&chassis_toe_vel_pid, g_chassis.current_vel - g_chassis.target_vel);
    float toe_ang_output = -PID_Output(&chassis_toe_angle_pid, balance_ang - g_chassis.current_pitch);

    float toe_left_target_vel = toe_ang_output + toe_vel_output;
    float toe_right_target_vel = toe_ang_output + toe_vel_output;
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
}
void Chassis_Enable()
{
    if (enabled_count < 50) {
        Leg_Enable();
    }
    Toe_Enable();
}

void Chassis_Disable()
{
    enabled_count = 0;
    Leg_Disable();
    //Toe_Disable();
}

void Chassis_LQRCtrl(Chassis_t *chassis)
{
    // chassis->left_leg.target_leg_virtual_torq = K11 * chassis->left_leg.current_disp + K12 * chassis->left_leg.current_vel
    //                                     + K13 * chassis->left_leg.current_theta + K14 * chassis->left_leg.current_theta_dot 
    //                                     + K15 * chassis->current_pitch + K16 * chassis->current_pitch_dot;
    
    // chassis->left_leg.target_leg_virtual_torq = K11 * chassis->right_leg.current_disp + K12 * chassis->right_leg.current_vel
    //                                     + K13 * chassis->right_leg.current_theta + K14 * chassis->right_leg.current_theta_dot 
    //                                     + K15 * chassis->current_pitch + K16 * chassis->current_pitch_dot;
    // __MAX_LIMIT(chassis->left_leg.target_leg_virtual_torq, -1, 1);
    // __MAX_LIMIT(chassis->right_leg.target_leg_virtual_torq, -1, 1);

    // chassis->left_leg.target_wheel_torq = K21 * chassis->left_leg.current_disp + K22 * chassis->left_leg.current_vel
    //                                     + K23 * chassis->left_leg.current_theta + K24 * chassis->left_leg.current_theta_dot 
    //                                     + K25 * chassis->current_pitch + K26 * chassis->current_pitch_dot;
    // chassis->right_leg.target_wheel_torq = K21 * chassis->right_leg.current_disp + K22 * chassis->right_leg.current_vel
    //                                     + K23 * chassis->right_leg.current_theta + K24 * chassis->right_leg.current_theta_dot 
    //                                     + K25 * chassis->current_pitch + K26 * chassis->current_pitch_dot;
    //Chassis_ToeControl();
    chassis->target_left_leg_virtual_force = (PID_Output(&chassis->left_leg_length_pid, chassis->target_height - chassis->left_leg.length));// + ROBOT_WEIGHT / 2);
    chassis->target_right_leg_virtual_force = PID_Output(&chassis->right_leg_length_pid, chassis->target_height - chassis->right_leg.length);//+ ROBOT_WEIGHT / 2;
    chassis->left_leg.target_leg_virtual_torq = PID_Output(&chassis_left_leg_angle, PI / 2 - chassis->left_leg.phi0);
    chassis->right_leg.target_leg_virtual_torq = PID_Output(&chassis_right_leg_angle, PI / 2 - chassis->right_leg.phi0);

    Leg_VMC(&chassis->left_leg, 0*chassis->target_left_leg_virtual_force, 0*chassis->left_leg.target_leg_virtual_torq);
    Leg_VMC(&chassis->right_leg, chassis->target_right_leg_virtual_force, chassis->right_leg.target_leg_virtual_torq);
}

void Chassis_Send(Chassis_t *chassis)
{
    //Leg_CtrlLeg(0, 0, 0, PI);
    Leg_TorqCtrl(chassis->left_leg.torq4, chassis->right_leg.torq1, chassis->right_leg.torq4, chassis->left_leg.torq1);
    Toe_TorqCtrl(chassis->target_wheel_torq, chassis->target_wheel_torq);
    //Toe_VelCtrl(0, 0);
    //Toe_PosCtrl(0, 0);
}

void Chassis_Ctrl(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(4);
    Chassis_Init();
    while (1)
    {
        Chassis_Process(&g_remote);
        if (g_chassis.enabled) 
        {    
            Chassis_Enable();
            Chassis_LQRCtrl(&g_chassis);
            Chassis_Send(&g_chassis);
            //Toe_TorqCtrl(0.05f, 0.05f);
        }
        else 
            Chassis_Disable();
            Toe_TorqCtrl(0.0f, 0.0f);
        
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
