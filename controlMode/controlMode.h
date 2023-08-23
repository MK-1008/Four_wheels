#ifndef __CONTROLMODE_H__
#define __CONTROLMODE_H__
#ifdef __cplusplus
extern "C" {
#endif
#include "can.h"
#define R_MOTOR1_ID 0x235
#define R_MOTOR2_ID 0x235
#define forward 0.6
#define backward -0.8

#define angle_in_degrees 80.00
#define left 1.0
#define right  -1.0
#define speed 1
void control_vehicle_with_remote(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RXHeader, uint8_t *RXmessage);
void CalculateMotion(float speed_mps, float angular_velocity_radps, float *left_mps, float *right_mps);
void control_vehicle_with_pc(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RXHeader, uint8_t *RXmessage);
#ifdef __cplusplus
}
#endif

#endif /* __CONTROLMODE_H__ */
