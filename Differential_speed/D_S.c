#include "D_S.h"
#include "DriveMotor.h"
#include "can.h"
#include "stm32f4xx_it.h"
#include <math.h>

// 全局变量，用于在Keil5中观察

float velocities[4];
float steering_angles[4];
VehicleMotion motion1;
uint16_t motor_id[4] = {  FRONT_LEFT_MOTOR_ID, FRONT_RIGHT_MOTOR_ID, REAR_LEFT_MOTOR_ID, REAR_RIGHT_MOTOR_ID};

int sendQueueIndex = 0;
Message sendQueue[4];

// 轮子速度和转向角的计算
void calculateWheelCommands(Twist curr_cmd_twist, float* velocities, float* steering_angles) {
    float vel_steering_offset = (curr_cmd_twist.ang * WHEEL_STEERING_Y_OFFSET) / WHEEL_RADIUS;
	  float sign = curr_cmd_twist.lin_x >= 0 ? 1.0f : -1.0f;

    velocities[0] = WHEEL_RADIUS * sign * sqrtf(powf(curr_cmd_twist.lin_x - curr_cmd_twist.ang * STEERING_TRACK / 2, 2) +
                                  powf(WHEEL_BASE * curr_cmd_twist.ang / 2.0f, 2)) / WHEEL_RADIUS - vel_steering_offset;
    velocities[1] = WHEEL_RADIUS * sign * sqrtf(powf(curr_cmd_twist.lin_x + curr_cmd_twist.ang * STEERING_TRACK / 2, 2) +
                                  powf(WHEEL_BASE * curr_cmd_twist.ang / 2.0f, 2)) / WHEEL_RADIUS - vel_steering_offset;
    velocities[2] = WHEEL_RADIUS * sign * sqrtf(powf(curr_cmd_twist.lin_x + curr_cmd_twist.ang * STEERING_TRACK / 2, 2) +
                                  powf(WHEEL_BASE * curr_cmd_twist.ang / 2.0f, 2)) / WHEEL_RADIUS + vel_steering_offset;
    velocities[3] = WHEEL_RADIUS * sign * sqrtf(powf(curr_cmd_twist.lin_x - curr_cmd_twist.ang * STEERING_TRACK / 2, 2) +
                                  powf(WHEEL_BASE * curr_cmd_twist.ang / 2.0f, 2)) / WHEEL_RADIUS + vel_steering_offset;

    steering_angles[0] = atanf(curr_cmd_twist.ang * WHEEL_BASE / (2.0f * curr_cmd_twist.lin_x - curr_cmd_twist.ang * STEERING_TRACK));
    steering_angles[1] = -steering_angles[0];
    steering_angles[2] = steering_angles[0];
    steering_angles[3] = -steering_angles[0];
}

// 设置车辆速度的函数
void SetVehicleSpeed(CAN_HandleTypeDef* hcan, Twist curr_cmd_twist) 
{
  calculateWheelCommands(curr_cmd_twist, velocities, steering_angles);
	for (int i = 0; i < 4; i++) {
        sendQueue[i].motor_id = motor_id[i];
        sendQueue[i].motion.speed_mps = velocities[i];
        sendQueue[i].motion.angular_velocity_radps = steering_angles[i];
	}
    sendQueueIndex = 0;
    // 发送第一个消息
    PowerMotor_SendMessage1(hcan, sendQueue[sendQueueIndex].motor_id, sendQueue[sendQueueIndex].motion);
    // 使用VehicleMotion结构体发送速度和角速度
}
