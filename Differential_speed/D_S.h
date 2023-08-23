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

// 速度和角速度结构体
typedef struct {
    float lin_x; // 线速度
    float ang;   // 角速度
} Twist;
extern float velocities[4]; // 需要监视的全局变量
extern float steering_angles[4]; // 需要监视的全局变量
extern VehicleMotion motion1;
extern uint16_t motor_id[4];

//车辆can_ID
#define FRONT_LEFT_MOTOR_ID 0x123
#define FRONT_RIGHT_MOTOR_ID 0x124
#define REAR_LEFT_MOTOR_ID 0x125
#define REAR_RIGHT_MOTOR_ID 0x126

// 车辆参数
#define WHEEL_RADIUS            26.0f  // 轮子半径，单位cm
#define WHEEL_BASE              12.2f  // 车辆轮距，单位cm
#define STEERING_TRACK          12.2f  // 车辆轴距，单位cm
#define WHEEL_STEERING_Y_OFFSET 3.0f  // 轮子在Y轴上的偏移量，单位cm

#define LEFT_MOTOR_ID 0x22A
#define RIGHT_MOTOR_ID 0x22A
#define MAX_SPEED_MPS 0.50f
#define WHEEL_DISTANCE_MM 500.0     // 驱动轮直径，单位：毫米
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
