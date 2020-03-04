#ifndef VIRTUAL_MOTOR_H
#define VIRTUAL_MOTOR_H
#include "stm32f4xx_can.h"

#define VM_FEEDBACK_HZ 100	//电机反馈频率，最好能被1000整除，实际反馈略小

#define VM_ACC 0	//如果控制信息是加速度，VM_CONTROL_TYPE赋值这个
#define VM_RPM 1	//如果控制信息是转速，VM_CONTROL_TYPE赋值这个
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

//电机信息结构体
typedef struct
{
	s16 position;	//位置反馈 
	s16 maxPos;		//反馈的最大位置 大疆电机设置成8191
	s16 minPos;		//反馈的最小位置 大疆电机设置成0
	s16 rpm;		//速度反馈
	s16 id;			//电机id 0x201-0x20C
	s16 maxRPM;		//最大转速
	s16 acc;		//加速度
} vm_virtualMotor_t;

typedef struct
{
	s16 vmFeedbackHz;			//反馈频率
	void (*vmCan1SendFun)(CanTxMsg msg);	//can1总线发送数据的函数指针
	void (*vmCan2SendFun)(CanTxMsg msg);	//can2总线发送数据的函数指针
	
	vm_virtualMotor_t vmCan1MotorList[VM_CAN1_MOTOR_SUM];	//挂载到can1总线的电机，需要调用电机初始化函数
	vm_virtualMotor_t vmCan2MotorList[VM_CAN2_MOTOR_SUM];	//挂载到can2总线的电机，需要调用电机初始化函数
	
} virtualMotorTask_t;

void virtualMotorTask(void*);

//canNum : 0 代表can1 1 代表can2
//msg:收到的can控制信息，0x1ff 或者 0x200， 通过发送过来的can信号修改对应的电机结构体的数据
void vmCtrlMsgRecieve(CanRxMsg *msg, u8 canNum);

#endif