#include "virtualMotor.h"
#include "FreeRTOS.h"
#include "task.h"
#ifndef NULL
#define NULL 0
#endif

s16 intLimit(s16 value, s16 maxValue);
s16 loopIntLimit(s16 value, s16 minVal, s16 maxVal);

virtualMotorTask_t virtualMotorTaskStructure;
void vmMotorUpdata(vm_virtualMotor_t *theMotor, s16 updataHz);
void vmSendMotorMsg(vm_virtualMotor_t *theMotor, void (*vmSendFun)(CanTxMsg msg));
void vmCan1SendFun(CanTxMsg msg);
void vmCan2SendFun(CanTxMsg msg);
void virtualMotorInit(vm_virtualMotor_t *motor, s16 id, s16 maxPos, s16 minPos, s16 maxRpm);
void virtualMotorTaskInit();
void vmMotorCtrlMsgReviece(s16 motorId, u8 canNum, s16 msg);

void vmCtrlMsgRecieve(CanRxMsg *msg, u8 canNum)
{
	if(msg == NULL)
		return;
	
	//根据报文id确定数据部分对应的电机ID
	s16 recieveId;
	if(msg->StdId == 0x1FF)
		recieveId = 0x205;
	else if(msg->StdId == 0x200)
		recieveId = 0x201;
	else if(msg->StdId == 0x2FF)
		recieveId = 0x209;
	else
		return;
		
	for(s16 i = 0; i < msg->DLC / 2; i++)
	{ //根据报文信息，控制电机，修改电机中的数据
		
			
		s16 value = (msg->Data[2*i] << 8) | msg->Data[2*i+1];
		vmMotorCtrlMsgReviece(recieveId + i,canNum ,value);
	}
}

//根据总线和电机id修改电机信息
//motorId 要修改的电机id
//canNum:挂载的can总线
//msg:can发送过来的信息，可能是加速度也可能是转速，取决于宏定义VM_CONTROL_TYPE
void vmMotorCtrlMsgReviece(s16 motorId, u8 canNum, s16 msg)
{
	vm_virtualMotor_t *start;
	s16 motorLength;
	if(canNum == 0)
	{
		start = virtualMotorTaskStructure.vmCan1MotorList;
		motorLength = VM_CAN1_MOTOR_SUM;
	}
	else if(canNum == 1)
	{
		start = virtualMotorTaskStructure.vmCan2MotorList;
		motorLength = VM_CAN2_MOTOR_SUM;		
	}
	else
		return;
	
	for(vm_virtualMotor_t *iterator = start; iterator < start + motorLength; iterator++)
	{
		if(iterator->id == motorId)
		{
			#if VM_CONTROL_TYPE == VM_ACC
			iterator->acc = msg;
			#elif VM_CONTROL_TYPE == VM_RPM
			iterator->rpm = msg;
			iterator->rpm = intLimit(iterator->rpm, iterator->maxRPM);
			#endif
			return;
		}
	}
}

//FreeRTOS调用的任务
void virtualMotorTask(void *pvParameters)
{
	virtualMotorTaskInit();
	while(1)
	{
		for(vm_virtualMotor_t *iterator = virtualMotorTaskStructure.vmCan1MotorList; iterator < virtualMotorTaskStructure.vmCan1MotorList + VM_CAN1_MOTOR_SUM; iterator++)
		{
			vmMotorUpdata(iterator, virtualMotorTaskStructure.vmFeedbackHz);
			vmSendMotorMsg(iterator, virtualMotorTaskStructure.vmCan1SendFun);
		}
		
		for(vm_virtualMotor_t *iterator = virtualMotorTaskStructure.vmCan2MotorList; iterator < virtualMotorTaskStructure.vmCan2MotorList + VM_CAN2_MOTOR_SUM; iterator++)
		{
			vmMotorUpdata(iterator, virtualMotorTaskStructure.vmFeedbackHz);
			vmSendMotorMsg(iterator, virtualMotorTaskStructure.vmCan2SendFun);
		}
		
		vTaskDelay(1000/virtualMotorTaskStructure.vmFeedbackHz);
	}
}

void virtualMotorTaskInit()
{
	//调用virtualMotorInit函数 初始化各个电机
	virtualMotorInit(virtualMotorTaskStructure.vmCan1MotorList + VM_CHASSIS_MOTOR_1, 0x201, 8191, 0, 1000);
	virtualMotorInit(virtualMotorTaskStructure.vmCan1MotorList + VM_CHASSIS_MOTOR_2, 0x202, 8191, 0, 1000);
	virtualMotorInit(virtualMotorTaskStructure.vmCan1MotorList + VM_CHASSIS_MOTOR_3, 0x203, 8191, 0, 1000);
	virtualMotorInit(virtualMotorTaskStructure.vmCan1MotorList + VM_CHASSIS_MOTOR_4, 0x204, 8191, 0, 1000);	
	virtualMotorInit(virtualMotorTaskStructure.vmCan1MotorList + VM_GIMBAL_MOTOR_YAW, 0x205, 8191, 0, 1000);	
	virtualMotorInit(virtualMotorTaskStructure.vmCan1MotorList + VM_GIMBAL_MOTOR_PITCH, 0x206, 8191, 0, 1000);
	
	virtualMotorInit(virtualMotorTaskStructure.vmCan2MotorList + VM_SHOOT_LEFT, 0x205, 8191, 0, 1000);
	virtualMotorInit(virtualMotorTaskStructure.vmCan2MotorList + VM_SHOOT_RIGHE, 0x206, 8191, 0, 1000);
	virtualMotorInit(virtualMotorTaskStructure.vmCan2MotorList + VM_SHOOT_PLATE, 0x207, 8191, 0, 1000);
	//配置virtualMotorTaskStructure结构体
	virtualMotorTaskStructure.vmFeedbackHz = VM_FEEDBACK_HZ;	//反馈的频率，最好能被1000整除
	virtualMotorTaskStructure.vmCan2SendFun = vmCan2SendFun;	//发送can2信息的函数指针赋值
	virtualMotorTaskStructure.vmCan1SendFun = vmCan1SendFun;	//发送can1信息的函数指针赋值
	
}

void vmMotorUpdata(vm_virtualMotor_t *theMotor, s16 updataHz)
{
	s16 lastRPM = theMotor->rpm;
	theMotor->rpm += theMotor->rpm + theMotor->acc*(1.0/updataHz);
	theMotor->rpm = intLimit(theMotor->rpm, theMotor->maxRPM);
	
	theMotor->position += (theMotor->rpm + lastRPM)/2.0/60.0/updataHz*8191;
	theMotor->position = loopIntLimit(theMotor->position, theMotor->minPos, theMotor->maxPos);
}

void vmSendMotorMsg(vm_virtualMotor_t *theMotor, void (*vmSendFun)(CanTxMsg msg))
{
	CanTxMsg msg;
	msg.StdId = theMotor->id;
	msg.IDE=CAN_ID_STD;		  
	msg.RTR=CAN_RTR_DATA;
	msg.DLC = 8;
	msg.Data[0] = (theMotor->position >> 8)&0xFF;
	msg.Data[1] = (theMotor->position)&0xFF;
	msg.Data[2] = (theMotor->rpm >> 8)&0xFF;
	msg.Data[3] = (theMotor->rpm)&0xFF;
	msg.Data[4] = (theMotor->acc >> 8)&0xFF;
	msg.Data[5] = (theMotor->acc)&0xFF;
	msg.Data[6] = 0;
	msg.Data[7] = 0;
	
	if(vmSendFun != NULL)
		vmSendFun(msg);
}

//初始化一个虚拟的电机
//motor:要初始化的电机的结构体指针
//maxPos:电机反馈的最大位置，大疆的电机设置成8191
//minPos：反馈的最小位置，大疆的电机设置成0
//maxRpm：转速上限
void virtualMotorInit(vm_virtualMotor_t *motor, s16 id, s16 maxPos, s16 minPos, s16 maxRpm)
{
	motor->id = id;
	motor->maxPos = maxPos;
	motor->minPos = minPos;
	motor->maxRPM = maxRpm;
	motor->acc = 0;
	motor->position = 0;
	motor->rpm = 0;
}

s16 intLimit(s16 value, s16 maxValue)
{
	if(value < -maxValue)
		value = -maxValue;
	else if(value > maxValue)
		value = maxValue;
	
	return value;
}

s16 loopIntLimit(s16 value, s16 minVal, s16 maxVal)
{
	while(value < minVal)
	{
		value += maxVal - minVal;
	}
	while(value > maxVal)
	{
		value -= maxVal - minVal;
	}
	return value;
}

void vmCan2SendFun(CanTxMsg msg)
{

	while(CAN_Transmit(CAN2, &msg) == CAN_TxStatus_NoMailBox);
}

void vmCan1SendFun(CanTxMsg msg)
{

	while(CAN_Transmit(CAN2, &msg) == CAN_TxStatus_NoMailBox);
}
