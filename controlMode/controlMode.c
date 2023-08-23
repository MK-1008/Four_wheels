#include <DriveMotor.h>
#include "controlMode.h"
#include "main.h"
#include "can.h"
#include "D_S.h"
#include "math.h"
#include <stdio.h>
/*这个是手动驾驶模式
 遥控器实现车辆的前进，后退，左转 右转等控制*/

const float ANGULAR_VELOCITY_INCREMENT_RADPS = 0.001f;
const float SPEED_INCREMENT_MPS = 0.00001f;
float speed_mps = 0.0f;  // 初始速度为0
float angular_velocity_radps = 0.0f;  // 初始角速度为0
float speed_x; // 单位: cm/s
float speed_y; // 单位: cm/s
float angular_velocity; //单位: 弧度/秒
int16_t velocity_x;
int16_t velocity_y;
int16_t angular_velocity_z;
uint8_t flagx,flagy;
float sign;


void control_vehicle_with_remote(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RXHeader, uint8_t *RXmessage)
{
	 //定义一个结构体
	  Twist Motor_real;
	 // 标记，判断是否有按键被按下
    char  anyKeyPressed = 0;
	
    // 检查是否接收到特定的CAN ID
    if (RXHeader->StdId == 0x401) 
		{
			//前进模式
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
			{
				HAL_Delay(10);
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
				{
					Motor_real.lin_x=100;
					Motor_real.ang=0;
					anyKeyPressed = 1;
				}
			}
			SetVehicleSpeed(&hcan1,Motor_real);
			
			//后退模式
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
			{
				HAL_Delay(10);
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
				{
					Motor_real.lin_x=-100;
					Motor_real.ang=0;
					anyKeyPressed = 1;
				}
			}
			SetVehicleSpeed(&hcan1,Motor_real);
			
			//左转模式
			
			//右转模式
			
			//左漂模式（定义左漂方向为负）
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
			{
				HAL_Delay(10);
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
				{
					Motor_real.lin_x=100;
					Motor_real.ang=-90;
					anyKeyPressed = 1;
				}
			}
			SetVehicleSpeed(&hcan1,Motor_real);
			
			//右漂模式（定义左漂方向为负）
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
			{
				HAL_Delay(10);
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
				{
					Motor_real.lin_x=100;
					Motor_real.ang=90;
					anyKeyPressed = 1;
				}
			}
			SetVehicleSpeed(&hcan1,Motor_real);
		}
			
    // 如果没有任何按键被按下，设置速度和角速度为0
    if (!anyKeyPressed) 
		{
        Motor_real.lin_x = 0.0f;
        Motor_real.ang = 0.0f;
    }
		


//    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
//        // 前进按键：增加速度
//        speed_mps += SPEED_INCREMENT_MPS;
//        anyKeyPressed = 1;
//    } 
//    else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)) {
//        // 后退按键：减少速度
//        speed_mps -= SPEED_INCREMENT_MPS;
//        anyKeyPressed = 1;
//    } 
//    else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)) {
//        // 左转按键：增加角速度
//        angular_velocity_radps += ANGULAR_VELOCITY_INCREMENT_RADPS;
//        anyKeyPressed = 1;
//    } 
//    else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)) {
//        // 右转按键：减少角速度
//        angular_velocity_radps -= ANGULAR_VELOCITY_INCREMENT_RADPS;
//        anyKeyPressed = 1;
//    }


}
void control_vehicle_with_pc(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RXHeader, uint8_t *RXmessage) {
	  
	//定义一个处理之前的结构体
	  Twist Motor_former;
	
    // 检查是否接收到特定的CAN ID
    if (RXHeader->StdId == 0x401) {
        // 解析接收到的数据
			
				// 单位: cm/s
				//解析x方向的速度
        velocity_x = (RXmessage[1] << 8) | RXmessage[0];
				flagx=(RXmessage[1] >> 7) & 0x01;
			  speed_x = (float)velocity_x *100 / 32767;
			
				//解析y方向的速度
        velocity_y = (RXmessage[3] << 8) | RXmessage[2];
				flagy=(RXmessage[3] >> 7) & 0x01;
			  speed_y = -(float)velocity_y *100 / 32767;
			
				if(flagy==1){
					sign = 1.0f;
				}
				else{
					sign = -1.0f;					
				}

				//解析z方向的速度
        angular_velocity_z = (RXmessage[5] << 8) | RXmessage[4];
				angular_velocity = (float)angular_velocity_z*100 / 32767; // 单位: 弧度/秒 

				Motor_former.ang=angular_velocity;
				Motor_former.lin_x= sqrtf(speed_x * speed_x + speed_y * speed_y);
				// 使用这些值控制您的车辆
				SetVehicleSpeed(&hcan1,Motor_former);
    }
}

//#define SPEED_CONVERSION_FACTOR 0.01f // 实际转换因子(单位是厘米/秒)
//#define ANGLE_CONVERSION_FACTOR 1000.0f // 角速度单位是0.001弧度/秒，所以转换因子是1000

//// 处理CAN消息的函数
//void control_vehicle_with_pc(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RXHeader, uint8_t *RXmessage)
//{
//    // 假设COB_ID_1是CAN1的ID，COB_ID_2是CAN2的ID
//    #define COB_ID_1 0x186
//    #define COB_ID_2 0x103

//    if (RXHeader->ExtId == COB_ID_1 || RXHeader->ExtId == COB_ID_2)
//    {
//        // 如果接收到的ID是我们关心的ID，那么我们就进行处理

//        uint8_t vehicle_status = RXmessage[0];
//        uint16_t target_speed = (RXmessage[1] << 8) | RXmessage[2];
//        uint16_t target_angular_velocity = (RXmessage[3] << 8) | RXmessage[4];  

//        // 根据协议进行状态解析
//        uint8_t right_turn_signal = vehicle_status & 0x80;
//        uint8_t left_turn_signal = vehicle_status & 0x40;
//        uint8_t release_brake = vehicle_status & 0x20;
//        uint8_t pull_brake = vehicle_status & 0x10;
//        uint8_t steering_enable = vehicle_status & 0x08;
//        uint8_t speed_enable = vehicle_status & 0x04;
//        uint8_t emergency_stop = vehicle_status & 0x02;
//        uint8_t control_enable = vehicle_status & 0x01;

//        // 根据状态调整速度和角度
//        if (emergency_stop || !control_enable) {
//            speed_mps = 0.0f;
//            angular_velocity_radps = 0.0f;
//        } else if (speed_enable) {
//            speed_mps = (float)target_speed / SPEED_CONVERSION_FACTOR; // SPEED_CONVERSION_FACTOR 是速度转换因子
//            angular_velocity_radps = (float)target_angular_velocity / ANGLE_CONVERSION_FACTOR; // ANGLE_CONVERSION_FACTOR 是角速度转换因子
//        }

//        if (RXHeader->ExtId == COB_ID_1) {
//            // 这是来自CAN1的信息，所以我们将这些信息应用于CAN1的电机
//            //SetVehicleSpeedForMotor1(&hcan1, speed_mps, angular_velocity_radps);
//        }
//				else if (RXHeader->ExtId == COB_ID_2) {
//            // 这是来自CAN2的信息，所以我们将这些信息应用于CAN2的电机
//            //SetVehicleSpeedForMotor2(&hcan2, speed_mps, angular_velocity_radps);
//        }
//    }
//}



