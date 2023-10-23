#include "thigh.h"

DM4310_Info_t thigh[4];
PID_t thigh_pos_pid[4], thigh_vel_pid[4];

void Thigh_Init(void);
void Thigh_Enable(void);
void Thigh_Disable(void);
void Thigh_Ctrl(float height, float angle);
void Thigh_CtrlLeg(float left_front, float right_front, float right_rear, float left_rear);
void Thigh_InverseKinematics(float height, float leg_angle, float *thigh_1, float *thigh_2);

void Thigh_Init()
{
    for (int i = 0; i < 4; i++)
    {
        PID_Init(&thigh_pos_pid[i], 2.5f, 0.005f, 250.0f, 10.0f, 10.0f, 0.0f);
        PID_Init(&thigh_vel_pid[i], 2.5f, 0.005f, 250.0f, 10.0f, 10.0f, 0.0f);
    }
}

void Thigh_Enable()
{
    DM4310_EnableMotor(&hcan1, LEFT_FRONT_ID);
    DM4310_EnableMotor(&hcan1, RIGHT_FRONT_ID);
    DM4310_EnableMotor(&hcan1, RIGHT_REAR_ID);
    DM4310_EnableMotor(&hcan1, LEFT_REAR_ID);
}

void Thigh_Disable()
{
    DM4310_DisableMotor(&hcan1, LEFT_FRONT_ID);
    DM4310_DisableMotor(&hcan1, RIGHT_FRONT_ID);
    DM4310_DisableMotor(&hcan1, RIGHT_REAR_ID);
    DM4310_DisableMotor(&hcan1, LEFT_REAR_ID);
    PID_Reset(&thigh_pos_pid[0]);
    PID_Reset(&thigh_pos_pid[1]);
    PID_Reset(&thigh_pos_pid[2]);
    PID_Reset(&thigh_pos_pid[3]);
}

void Thigh_Ctrl(float height, float angle)
{
    float target_thigh_1, target_thigh_2, target_thigh_3, target_thigh_4 = 0;
    Thigh_InverseKinematics(height, angle, &target_thigh_1, &target_thigh_2);
    Thigh_InverseKinematics(height, (PI - angle), &target_thigh_3, &target_thigh_4);
    Thigh_CtrlLeg(target_thigh_2, target_thigh_3, target_thigh_4, target_thigh_1);
}

void Thigh_CtrlLeg(float left_front, float right_front, float right_rear, float left_rear)
{
    float torq_lf = PID(&thigh_pos_pid[0], ((left_front + LEFT_FRONT_OFFSET) - thigh[0].pos));
    float torq_rf = PID(&thigh_pos_pid[1], ((right_front + RIGHT_FRONT_OFFSET) - thigh[1].pos));
    float torq_rr = PID(&thigh_pos_pid[2], ((right_rear + RIGHT_REAR_OFFSET) - thigh[2].pos));
    float torq_lr = PID(&thigh_pos_pid[3], ((left_rear + LEFT_REAR_OFFSET) - thigh[3].pos));
    __MAX_LIMIT(torq_lf, -8, 8);
    __MAX_LIMIT(torq_rf, -8, 8);
    __MAX_LIMIT(torq_rr, -8, 8);
    __MAX_LIMIT(torq_lr, -8, 8);
    DM4310_CtrlMIT(&hcan1, LEFT_FRONT_ID, 0, 0, 0, 0, torq_lf);
    DM4310_CtrlMIT(&hcan1, RIGHT_FRONT_ID, 0, 0, 0, 0, torq_rf);
    DM4310_CtrlMIT(&hcan1, RIGHT_REAR_ID, 0, 0, 0, 0, torq_rr);
    DM4310_CtrlMIT(&hcan1, LEFT_REAR_ID, 0, 0, 0, 0, torq_lr);
}

void Thigh_InverseKinematics(float height, float leg_angle, float *thigh_1, float *thigh_2)
{
    float x_toe = height / (tan(leg_angle) == 0 ? 0.000001f : tan(leg_angle));
    float y_toe = height;

    float a_1 = 2 * (-HALF_THIGH_DISTANCE - x_toe) * THIGH_LENGTH;
    float b = -2 * y_toe * THIGH_LENGTH;
    float c_1 = pow(-HALF_THIGH_DISTANCE - x_toe, 2) + pow(y_toe, 2) + pow(THIGH_LENGTH, 2) - pow(CALF_LENGTH, 2);

    x_toe = -x_toe;
    float a_2 = 2 * (-HALF_THIGH_DISTANCE - x_toe) * THIGH_LENGTH;
    float c_2 = pow(-HALF_THIGH_DISTANCE - x_toe, 2) + pow(y_toe, 2) + pow(THIGH_LENGTH, 2) - pow(CALF_LENGTH, 2);

    *thigh_1 = 2 * atan2(-b + sqrt(pow(a_1, 2) + pow(b, 2) - pow(c_1, 2)), c_1 - a_1);
    *thigh_2 = -(2 * atan2(-b + sqrt(pow(a_2, 2) + pow(b, 2) - pow(c_2, 2)), c_2 - a_2) - PI);
}
