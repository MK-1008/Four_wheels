#include <DriveMotor.h>
#include "controlMode.h"
#include "main.h"
#include "can.h"
#include "D_S.h"
#include "math.h"
#include <stdio.h>
/*������ֶ���ʻģʽ
 ң����ʵ�ֳ�����ǰ�������ˣ���ת ��ת�ȿ���*/

const float ANGULAR_VELOCITY_INCREMENT_RADPS = 0.001f;
const float SPEED_INCREMENT_MPS = 0.00001f;
float speed_mps = 0.0f;  // ��ʼ�ٶ�Ϊ0
float angular_velocity_radps = 0.0f;  // ��ʼ���ٶ�Ϊ0
float speed_x; // ��λ: cm/s
float speed_y; // ��λ: cm/s
float angular_velocity; //��λ: ����/��
int16_t velocity_x;
int16_t velocity_y;
int16_t angular_velocity_z;
uint8_t flagx,flagy;
float sign;


void control_vehicle_with_remote(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RXHeader, uint8_t *RXmessage)
{
	 //����һ���ṹ��
	  Twist Motor_real;
	 // ��ǣ��ж��Ƿ��а���������
    char  anyKeyPressed = 0;
	
    // ����Ƿ���յ��ض���CAN ID
    if (RXHeader->StdId == 0x401) 
		{
			//ǰ��ģʽ
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
			
			//����ģʽ
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
			
			//��תģʽ
			
			//��תģʽ
			
			//��Ưģʽ��������Ư����Ϊ����
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
			
			//��Ưģʽ��������Ư����Ϊ����
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
			
    // ���û���κΰ��������£������ٶȺͽ��ٶ�Ϊ0
    if (!anyKeyPressed) 
		{
        Motor_real.lin_x = 0.0f;
        Motor_real.ang = 0.0f;
    }
		


//    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
//        // ǰ�������������ٶ�
//        speed_mps += SPEED_INCREMENT_MPS;
//        anyKeyPressed = 1;
//    } 
//    else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)) {
//        // ���˰����������ٶ�
//        speed_mps -= SPEED_INCREMENT_MPS;
//        anyKeyPressed = 1;
//    } 
//    else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)) {
//        // ��ת���������ӽ��ٶ�
//        angular_velocity_radps += ANGULAR_VELOCITY_INCREMENT_RADPS;
//        anyKeyPressed = 1;
//    } 
//    else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)) {
//        // ��ת���������ٽ��ٶ�
//        angular_velocity_radps -= ANGULAR_VELOCITY_INCREMENT_RADPS;
//        anyKeyPressed = 1;
//    }


}
void control_vehicle_with_pc(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RXHeader, uint8_t *RXmessage) {
	  
	//����һ������֮ǰ�Ľṹ��
	  Twist Motor_former;
	
    // ����Ƿ���յ��ض���CAN ID
    if (RXHeader->StdId == 0x401) {
        // �������յ�������
			
				// ��λ: cm/s
				//����x������ٶ�
        velocity_x = (RXmessage[1] << 8) | RXmessage[0];
				flagx=(RXmessage[1] >> 7) & 0x01;
			  speed_x = (float)velocity_x *100 / 32767;
			
				//����y������ٶ�
        velocity_y = (RXmessage[3] << 8) | RXmessage[2];
				flagy=(RXmessage[3] >> 7) & 0x01;
			  speed_y = -(float)velocity_y *100 / 32767;
			
				if(flagy==1){
					sign = 1.0f;
				}
				else{
					sign = -1.0f;					
				}

				//����z������ٶ�
        angular_velocity_z = (RXmessage[5] << 8) | RXmessage[4];
				angular_velocity = (float)angular_velocity_z*100 / 32767; // ��λ: ����/�� 

				Motor_former.ang=angular_velocity;
				Motor_former.lin_x= sqrtf(speed_x * speed_x + speed_y * speed_y);
				// ʹ����Щֵ�������ĳ���
				SetVehicleSpeed(&hcan1,Motor_former);
    }
}

//#define SPEED_CONVERSION_FACTOR 0.01f // ʵ��ת������(��λ������/��)
//#define ANGLE_CONVERSION_FACTOR 1000.0f // ���ٶȵ�λ��0.001����/�룬����ת��������1000

//// ����CAN��Ϣ�ĺ���
//void control_vehicle_with_pc(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RXHeader, uint8_t *RXmessage)
//{
//    // ����COB_ID_1��CAN1��ID��COB_ID_2��CAN2��ID
//    #define COB_ID_1 0x186
//    #define COB_ID_2 0x103

//    if (RXHeader->ExtId == COB_ID_1 || RXHeader->ExtId == COB_ID_2)
//    {
//        // ������յ���ID�����ǹ��ĵ�ID����ô���Ǿͽ��д���

//        uint8_t vehicle_status = RXmessage[0];
//        uint16_t target_speed = (RXmessage[1] << 8) | RXmessage[2];
//        uint16_t target_angular_velocity = (RXmessage[3] << 8) | RXmessage[4];  

//        // ����Э�����״̬����
//        uint8_t right_turn_signal = vehicle_status & 0x80;
//        uint8_t left_turn_signal = vehicle_status & 0x40;
//        uint8_t release_brake = vehicle_status & 0x20;
//        uint8_t pull_brake = vehicle_status & 0x10;
//        uint8_t steering_enable = vehicle_status & 0x08;
//        uint8_t speed_enable = vehicle_status & 0x04;
//        uint8_t emergency_stop = vehicle_status & 0x02;
//        uint8_t control_enable = vehicle_status & 0x01;

//        // ����״̬�����ٶȺͽǶ�
//        if (emergency_stop || !control_enable) {
//            speed_mps = 0.0f;
//            angular_velocity_radps = 0.0f;
//        } else if (speed_enable) {
//            speed_mps = (float)target_speed / SPEED_CONVERSION_FACTOR; // SPEED_CONVERSION_FACTOR ���ٶ�ת������
//            angular_velocity_radps = (float)target_angular_velocity / ANGLE_CONVERSION_FACTOR; // ANGLE_CONVERSION_FACTOR �ǽ��ٶ�ת������
//        }

//        if (RXHeader->ExtId == COB_ID_1) {
//            // ��������CAN1����Ϣ���������ǽ���Щ��ϢӦ����CAN1�ĵ��
//            //SetVehicleSpeedForMotor1(&hcan1, speed_mps, angular_velocity_radps);
//        }
//				else if (RXHeader->ExtId == COB_ID_2) {
//            // ��������CAN2����Ϣ���������ǽ���Щ��ϢӦ����CAN2�ĵ��
//            //SetVehicleSpeedForMotor2(&hcan2, speed_mps, angular_velocity_radps);
//        }
//    }
//}



