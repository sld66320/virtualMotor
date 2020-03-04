#ifndef VIRTUAL_MOTOR_H
#define VIRTUAL_MOTOR_H
#include "stm32f4xx_can.h"

#define VM_FEEDBACK_HZ 100	//�������Ƶ�ʣ�����ܱ�1000������ʵ�ʷ�����С

#define VM_ACC 0	//���������Ϣ�Ǽ��ٶȣ�VM_CONTROL_TYPE��ֵ���
#define VM_RPM 1	//���������Ϣ��ת�٣�VM_CONTROL_TYPE��ֵ���
#define VM_CONTROL_TYPE VM_RPM 


typedef enum
{
	VM_CHASSIS_MOTOR_1 = 0,
	VM_CHASSIS_MOTOR_2,
	VM_CHASSIS_MOTOR_3,
	VM_CHASSIS_MOTOR_4,
	VM_GIMBAL_MOTOR_YAW,
	VM_GIMBAL_MOTOR_PITCH,
	VM_CAN1_MOTOR_SUM,
} VM_CAN1_MOTOR_e;

typedef enum
{
	VM_SHOOT_LEFT = 0,
	VM_SHOOT_RIGHE,
	VM_SHOOT_PLATE,
	VM_CAN2_MOTOR_SUM,
} VM_CAN2_MOTOR_e;

//�����Ϣ�ṹ��
typedef struct
{
	s16 position;	//λ�÷��� 
	s16 maxPos;		//���������λ�� �󽮵�����ó�8191
	s16 minPos;		//��������Сλ�� �󽮵�����ó�0
	s16 rpm;		//�ٶȷ���
	s16 id;			//���id 0x201-0x20C
	s16 maxRPM;		//���ת��
	s16 acc;		//���ٶ�
} vm_virtualMotor_t;

typedef struct
{
	s16 vmFeedbackHz;			//����Ƶ��
	void (*vmCan1SendFun)(CanTxMsg msg);	//can1���߷������ݵĺ���ָ��
	void (*vmCan2SendFun)(CanTxMsg msg);	//can2���߷������ݵĺ���ָ��
	
	vm_virtualMotor_t vmCan1MotorList[VM_CAN1_MOTOR_SUM];	//���ص�can1���ߵĵ������Ҫ���õ����ʼ������
	vm_virtualMotor_t vmCan2MotorList[VM_CAN2_MOTOR_SUM];	//���ص�can2���ߵĵ������Ҫ���õ����ʼ������
	
} virtualMotorTask_t;

void virtualMotorTask(void*);

//canNum : 0 ����can1 1 ����can2
//msg:�յ���can������Ϣ��0x1ff ���� 0x200�� ͨ�����͹�����can�ź��޸Ķ�Ӧ�ĵ���ṹ�������
void vmCtrlMsgRecieve(CanRxMsg *msg, u8 canNum);

#endif