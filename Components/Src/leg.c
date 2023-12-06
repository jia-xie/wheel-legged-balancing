#include "leg.h"

DM4310_Info_t leg_motors[4];
PID_t leg_pos_pid[4], leg_vel_pid[4];

void Leg_Init(void);
void Leg_Enable(void);
void Leg_Disable(void);
void Leg_Ctrl(float height, float angle);
void Leg_CtrlLeg(float left_front, float right_front, float right_rear, float left_rear);
void Leg_InverseKinematics(float height, float leg_angle, float *leg_1, float *leg_2);

void Leg_Init()
{
    for (int i = 0; i < 4; i++)
    {
        PID_Init(&leg_pos_pid[i], 1.5f, 0.02f, 150.0f, 10.0f, 10.0f, 0.0f);
        PID_Init(&leg_vel_pid[i], 1.5f, 0.02f, 150.0f, 10.0f, 10.0f, 0.0f);
    }
}

void Leg_Enable()
{
    DM4310_EnableMotor(&hcan1, LEFT_FRONT_ID);
    DM4310_EnableMotor(&hcan1, RIGHT_FRONT_ID);
    DM4310_EnableMotor(&hcan1, RIGHT_REAR_ID);
    DM4310_EnableMotor(&hcan1, LEFT_REAR_ID);
}

void Leg_Disable()
{
    DM4310_DisableMotor(&hcan1, LEFT_FRONT_ID);
    DM4310_DisableMotor(&hcan1, RIGHT_FRONT_ID);
    DM4310_DisableMotor(&hcan1, RIGHT_REAR_ID);
    DM4310_DisableMotor(&hcan1, LEFT_REAR_ID);
    PID_Reset(&leg_pos_pid[0]);
    PID_Reset(&leg_pos_pid[1]);
    PID_Reset(&leg_pos_pid[2]);
    PID_Reset(&leg_pos_pid[3]);
}

void Leg_Ctrl(float height, float angle)
{
    float target_leg_1, target_leg_2, target_leg_3, target_leg_4 = 0;
    Leg_InverseKinematics(height, angle, &target_leg_1, &target_leg_2);
    Leg_InverseKinematics(height, (PI - angle), &target_leg_3, &target_leg_4);
    Leg_CtrlLeg(target_leg_2, target_leg_3, target_leg_4, target_leg_1);
}

void Leg_ForwardKinematics(Leg_t *leg, float phi1, float phi4, float phi1_dot, float phi4_dot)
{
    leg->phi1 = phi1;
    leg->phi1_dot = phi1_dot;
    leg->phi4 = phi4;
    leg->phi4_dot = phi4_dot;
}

void Leg_CtrlLeg(float left_front, float right_front, float right_rear, float left_rear)
{
    float torq_lf = PID_Output(&leg_pos_pid[0], ((left_front + LEFT_FRONT_OFFSET) - leg_motors[0].pos));
    float torq_rf = PID_Output(&leg_pos_pid[1], ((right_front + RIGHT_FRONT_OFFSET) - leg_motors[1].pos));
    float torq_rr = PID_Output(&leg_pos_pid[2], ((right_rear + RIGHT_REAR_OFFSET) - leg_motors[2].pos));
    float torq_lr = PID_Output(&leg_pos_pid[3], ((left_rear + LEFT_REAR_OFFSET) - leg_motors[3].pos));
    __MAX_LIMIT(torq_lf, -8, 8);
    __MAX_LIMIT(torq_rf, -8, 8);
    __MAX_LIMIT(torq_rr, -8, 8);
    __MAX_LIMIT(torq_lr, -8, 8);
    DM4310_CtrlMIT(&hcan1, LEFT_FRONT_ID, 0, 0, 0, 0, torq_lf);
    DM4310_CtrlMIT(&hcan1, RIGHT_FRONT_ID, 0, 0, 0, 0, torq_rf);
    DM4310_CtrlMIT(&hcan1, RIGHT_REAR_ID, 0, 0, 0, 0, torq_rr);
    DM4310_CtrlMIT(&hcan1, LEFT_REAR_ID, 0, 0, 0, 0, torq_lr);
}

void Leg_InverseKinematics(float height, float leg_angle, float *leg_1, float *leg_2)
{
    float x_toe = height / (tan(leg_angle) == 0 ? 0.000001f : tan(leg_angle));
    float y_toe = height;

    float a_1 = 2 * (-HALF_THIGH_DISTANCE - x_toe) * THIGH_LENGTH;
    float b = -2 * y_toe * THIGH_LENGTH;
    float c_1 = pow(-HALF_THIGH_DISTANCE - x_toe, 2) + pow(y_toe, 2) + pow(THIGH_LENGTH, 2) - pow(CALF_LENGTH, 2);

    x_toe = -x_toe;
    float a_2 = 2 * (-HALF_THIGH_DISTANCE - x_toe) * THIGH_LENGTH;
    float c_2 = pow(-HALF_THIGH_DISTANCE - x_toe, 2) + pow(y_toe, 2) + pow(THIGH_LENGTH, 2) - pow(CALF_LENGTH, 2);

    *leg_1 = 2 * atan2(-b + sqrt(pow(a_1, 2) + pow(b, 2) - pow(c_1, 2)), c_1 - a_1);
    *leg_2 = -(2 * atan2(-b + sqrt(pow(a_2, 2) + pow(b, 2) - pow(c_2, 2)), c_2 - a_2) - PI);
}
