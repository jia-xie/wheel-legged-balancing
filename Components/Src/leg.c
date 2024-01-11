#include "leg.h"

#include "robot_param.h"
DM4310_Info_t leg_motors[4];
PID_t leg_pos_pid[4], leg_vel_pid[4];

void Leg_Init(void);
void Leg_Enable(void);
void Leg_Disable(void);
void Leg_CtrlTorq(float lf, float rf, float rr, float lr);
void Leg_CtrlLeg(float left_front, float right_front, float right_rear, float left_rear);
void Leg_InverseKinematics(float height, float leg_angle, float *leg_1, float *leg_2);
void Leg_ForwardKinematics(Leg_t *leg, float phi1, float phi2, float phi1_dot, float phi2_dot);
void Leg_VMC(Leg_t *leg, float force, float torq);

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

void Leg_CtrlTorq(float lf, float rf, float rr, float lr)
{
    __MAX_LIMIT(lf, -8, 8);
    __MAX_LIMIT(rf, -8, 8);
    __MAX_LIMIT(rr, -8, 8);
    __MAX_LIMIT(lr, -8, 8);
    DM4310_CtrlMIT(&hcan1, LEFT_FRONT_ID , 0, 0, 0, 0, lf);
    DM4310_CtrlMIT(&hcan1, RIGHT_FRONT_ID, 0, 0, 0, 0, rf);
    DM4310_CtrlMIT(&hcan1, RIGHT_REAR_ID , 0, 0, 0, 0, rr);
    DM4310_CtrlMIT(&hcan1, LEFT_REAR_ID  , 0, 0, 0, 0, lr);
}

void Leg_ForwardKinematics(Leg_t *leg, float phi1, float phi4, float phi1_dot, float phi4_dot)
{
    leg->current_tick = SysTick->VAL;
    leg->phi1 = phi1;
    leg->phi1_dot = phi1_dot;
    leg->phi4 = phi4;
    leg->phi4_dot = phi4_dot;
    
    float x_B = -HALF_THIGH_DISTANCE + THIGH_LENGTH * cos(leg->phi1);
    float y_B = THIGH_LENGTH * sin(leg->phi1);
    float x_D = HALF_THIGH_DISTANCE + THIGH_LENGTH * cos(leg->phi4);
    float y_D = THIGH_LENGTH * sin(leg->phi4);

    float xD_minus_xB = x_D - x_B;
    float yD_minus_yB = y_D - y_B;

    float A = 2 * CALF_LENGTH * xD_minus_xB;
    float B = 2 * CALF_LENGTH * yD_minus_yB;
    float C = pow(xD_minus_xB, 2) + pow(yD_minus_yB, 2);
    
    leg->phi2 = 2 * atan2(B + sqrt(pow(A, 2) + pow(B, 2) - pow(C, 2)), A + C);
    
    float x_C = x_B + CALF_LENGTH * cos(leg->phi2);
    float y_C = y_B + CALF_LENGTH * sin(leg->phi2);

    leg->phi3 = atan2(y_C - y_D, x_C - x_D);

    leg->xe1 = xD_minus_xB;
    leg->ye1 = yD_minus_yB;
    leg->xe2 = x_D;
    leg->ye2 = y_D;
    leg->length = sqrt(pow(x_C, 2) + pow(y_C, 2));
    leg->phi0 = atan2(y_C, x_C);

    
    leg->phi0_dot = (leg->phi0 - leg->last_phi0) / (0.004f);

    leg->last_phi0 = leg->phi0;
    leg->last_tick = leg->current_tick;
}

void Leg_VMC(Leg_t *leg, float force, float torq)
{
    float leg_length = leg->length;
    float theta = leg->phi0 - PI/2;
    float phi1 = leg->phi1;
    float phi4 = leg->phi4;
    float xC_minus_xB = (-leg_length * sin(theta)) - (
        -HALF_THIGH_DISTANCE + THIGH_LENGTH * cos(phi1));
    float yC_minus_yB = (leg_length * cos(theta)) - THIGH_LENGTH * sin(phi1);
    float xC_minus_xD = (-leg_length * sin(theta)) - (
        HALF_THIGH_DISTANCE + THIGH_LENGTH * cos(phi4));;
    float yC_minus_yD = (leg_length * cos(theta)) - THIGH_LENGTH * sin(phi4);
    
    float M11 = (1/THIGH_LENGTH)*((-sin(theta)*xC_minus_xB+cos(theta)*yC_minus_yB)/
                                  (-sin(phi1)*xC_minus_xB+cos(phi1)*yC_minus_yB));
    float M12 = (leg_length/THIGH_LENGTH)*((-cos(theta)*xC_minus_xB-sin(theta)*yC_minus_yB)/
                                           (-sin(phi1)*xC_minus_xB+cos(phi1)*yC_minus_yB));
    float M21 = (1/THIGH_LENGTH)*((-sin(theta)*xC_minus_xD+cos(theta)*yC_minus_yD)/
                                  (-sin(phi4)*xC_minus_xD+cos(phi4)*yC_minus_yD));
    float M22 = (leg_length/THIGH_LENGTH)*((-cos(theta)*xC_minus_xD-sin(theta)*yC_minus_yD)/
                                           (-sin(phi4)*xC_minus_xD+cos(phi4)*yC_minus_yD));
    
    float one_over_deter = 1 / (M11 * M22 - M12 * M21);
    float J11 = one_over_deter * M22;
    float J12 = -one_over_deter * M12;
    float J21 = -one_over_deter * M21;
    float J22 = one_over_deter * M11;

    leg->torq1 = J11 * force + J21 * torq;
    leg->torq4 = J12 * force + J22 * torq;
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
