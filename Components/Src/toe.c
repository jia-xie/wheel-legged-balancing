#include "toe.h"

MF_MOTOR_INFO_t g_toe[2];

PID_t toe_vel_left_pid, toe_vel_right_pid;
PID_t leg_angle_pid, toe_state_pid;
PID_t toe_pos_left_pid, toe_pos_right_pid;

void Toe_Init(void);
void Toe_Enable(void);
void Toe_Disable(void);
void Toe_Ctrl(float left, float right);
void Toe_PosCtrl(float left, float right);


void Toe_Init()
{
    MF_Motor_DisableMotor(&hcan2, LEFT_TOE_ID);
    MF_Motor_DisableMotor(&hcan2, RIGHT_TOE_ID);
    PID_Init(&toe_vel_left_pid, 0.30f, 0.01f, 0.08f, 2000.0f, 600.0f, 0.0f);
    PID_Init(&toe_vel_right_pid, 0.30f, 0.01f, 0.08f, 2000.0f, 600.0f, 0.0f);
    PID_Init(&toe_pos_left_pid, 0.0015f, 0.0f, 0.05f, 0.4f, 0.0f, 0.0f);
    PID_Init(&toe_pos_right_pid, 0.0015f, 0.0f, 0.05f, 0.4f, 0.0f, 0.0f);
}

void Toe_Enable()
{
    if (!g_toe[0].enabled) 
        MF_Motor_EnableMotor(&hcan2, LEFT_TOE_ID);
    if (!g_toe[1].enabled) 
        MF_Motor_EnableMotor(&hcan2, RIGHT_TOE_ID);
}

void Toe_Disable()
{

    MF_Motor_DisableMotor(&hcan2, LEFT_TOE_ID);
    PID_Reset(&toe_vel_left_pid);
 

    MF_Motor_DisableMotor(&hcan2, RIGHT_TOE_ID);
    PID_Reset(&toe_vel_right_pid);


    
}

void Toe_VelCtrl(int32_t left, int32_t right)
{
    float torq_l = PID_Output(&toe_vel_left_pid, left - TOE_LEFT_DIR * g_toe[0].velocity);
    float torq_r = PID_Output(&toe_vel_right_pid, right - TOE_RIGHT_DIR *  g_toe[1].velocity);
    Toe_TorqCtrl(torq_l, torq_r);
}

void Toe_TorqCtrl(float left, float right)
{
    __MAX_LIMIT(left, -0.5f, 0.5f);
    __MAX_LIMIT(right, -0.5f, 0.5f);
    int16_t left_int = (left / MF4310_TORQ_CONST) * CURRENT_TO_CTRL_INT;
    int16_t right_int = (right / MF4310_TORQ_CONST) * CURRENT_TO_CTRL_INT;
    __MAX_LIMIT(left_int, -2000, 2000);
    __MAX_LIMIT(right_int, -2000, 2000);
    MF_Motor_TorqueCtrl(&hcan2, LEFT_TOE_ID, TOE_LEFT_DIR * left_int);
    MF_Motor_TorqueCtrl(&hcan2, RIGHT_TOE_ID, TOE_RIGHT_DIR * right_int);
}

void Toe_PosCtrl(float left, float right)
{
    float torq_left = PID_Output(&toe_pos_left_pid, left - g_toe[0].total_angle);
    float torq_right = PID_Output(&toe_pos_right_pid, right - g_toe[1].total_angle);
    Toe_TorqCtrl(torq_left, -torq_right);
}
