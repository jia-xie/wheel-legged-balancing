#include "toe.h"

MF_MOTOR_INFO_t toe[2];

PID_t toe_vel_left_pid, toe_vel_right_pid;
PID_t leg_angle_pid, toe_state_pid;

void Toe_Init(void);
void Toe_Enable(void);
void Toe_Disable(void);
void Toe_Ctrl(int32_t left, int32_t right);

void Toe_Init()
{
    MF_Motor_DisableMotor(&hcan1, LEFT_TOE_ID);
    MF_Motor_DisableMotor(&hcan1, RIGHT_TOE_ID);
    PID_Init(&toe_vel_left_pid, 0.30f, 0.01f, 0.02f, 2000.0f, 600.0f, 0.0f);
    PID_Init(&toe_vel_right_pid, 0.30f, 0.01f, 0.02f, 2000.0f, 600.0f, 0.0f);
}

void Toe_Enable()
{
    if (!toe[0].enabled) 
        MF_Motor_EnableMotor(&hcan1, LEFT_TOE_ID);
    if (!toe[1].enabled) 
        MF_Motor_EnableMotor(&hcan1, RIGHT_TOE_ID);
}

void Toe_Disable()
{

    MF_Motor_DisableMotor(&hcan1, LEFT_TOE_ID);
    PID_Reset(&toe_vel_left_pid);
 

    MF_Motor_DisableMotor(&hcan1, RIGHT_TOE_ID);
    PID_Reset(&toe_vel_right_pid);


    
}

void Toe_VelCtrl(int32_t left, int32_t right)
{
    float torq_l = PID(&toe_vel_left_pid, left - TOE_LEFT_DIR * toe[0].velocity);
    float torq_r = PID(&toe_vel_right_pid, right - TOE_RIGHT_DIR *  toe[1].velocity);
    Toe_TorqCtrl(torq_l, torq_r);
}

void Toe_TorqCtrl(int32_t left, int32_t right)
{
    __MAX_LIMIT(left, -2000, 2000);
    __MAX_LIMIT(right, -2000, 2000);
    MF_Motor_TorqueCtrl(&hcan1, LEFT_TOE_ID, TOE_LEFT_DIR * left);
    MF_Motor_TorqueCtrl(&hcan1, RIGHT_TOE_ID, TOE_RIGHT_DIR * right);
}
