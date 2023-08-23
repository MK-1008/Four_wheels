#ifndef __D_S_H__
#define __D_S_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "DriveMotor.h"
enum Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};

typedef struct {
    uint16_t motor_id;
    VehicleMotion motion;
} Message;

extern Message sendQueue[4];
extern int sendQueueIndex;

// �ٶȺͽ��ٶȽṹ��
typedef struct {
    float lin_x; // ���ٶ�
    float ang;   // ���ٶ�
} Twist;
extern float velocities[4]; // ��Ҫ���ӵ�ȫ�ֱ���
extern float steering_angles[4]; // ��Ҫ���ӵ�ȫ�ֱ���
extern VehicleMotion motion1;
extern uint16_t motor_id[4];

//����can_ID
#define FRONT_LEFT_MOTOR_ID 0x123
#define FRONT_RIGHT_MOTOR_ID 0x124
#define REAR_LEFT_MOTOR_ID 0x125
#define REAR_RIGHT_MOTOR_ID 0x126

// ��������
#define WHEEL_RADIUS            26.0f  // ���Ӱ뾶����λcm
#define WHEEL_BASE              12.2f  // �����־࣬��λcm
#define STEERING_TRACK          12.2f  // ������࣬��λcm
#define WHEEL_STEERING_Y_OFFSET 3.0f  // ������Y���ϵ�ƫ��������λcm

#define LEFT_MOTOR_ID 0x22A
#define RIGHT_MOTOR_ID 0x22A
#define MAX_SPEED_MPS 0.50f
#define WHEEL_DISTANCE_MM 500.0     // ������ֱ������λ������
void DifferentialDrive(CAN_HandleTypeDef* hcan1, CAN_HandleTypeDef* hcan2, enum Direction direction/*, float* current_angle*/);
void DriveMotorsWithDifferential(float linear_velocity, float angular_velocity);

void calculateWheelCommands(Twist curr_cmd_twist, float* velocities, float* steering_angles);
void SetVehicleSpeed(CAN_HandleTypeDef* hcan, Twist curr_cmd_twist);

//void SetVehicleSpeedForMotor1(CAN_HandleTypeDef* hcan1, float speed_mps, float angular_velocity_radps);
//void SetVehicleSpeedForMotor2(CAN_HandleTypeDef* hcan2, float speed_mps, float angular_velocity_radps);
void SetVehicleSpeedAndAngularVelocity(CAN_HandleTypeDef* hcan1, CAN_HandleTypeDef* hcan2, float speed_mps, float angular_velocity_radps);
#ifdef __cplusplus
}
#endif

#endif /* __D_S_H__ */
