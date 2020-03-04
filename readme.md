## 虚拟电机使用功能
可以接收大疆3808/gm6020系列电机的can控制报文，并根据报文修改电机的虚拟数据（转速、转子位置），然后根据电机反馈的can报文格式，以提前设置好的频率进行反馈。
（由于能力有限，程序可能对于未知的特殊情况缺乏处理，还望大家包含T T，希望疫情早日结束，早日能够开学备赛。）

## 使用方法
1. 在枚举VM_CAN1_MOTOR_e中加入can1总线中电机的名称，在VM_CAN2_MOTOR_e枚举中加入can2总线的电机名称。并以VM_CAN1_MOTOR_SUM/VM_CAN2_MOTOR_SUM这两个枚举名称结束，用来统计电机的总数目。例如：
``` c
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
```

2. 通过上述步骤添加完电机名称后，在virtualMotorTask_t结构体中会生成对应数目的电机数据结构体，分别存储在vmCan1MotorList/vmCan2MotorList这两个数组中，这些结构体存储了电机的转速，转字位置等信息，此步是方便大家理解，没有实际操作。
3. 在virtualMotorTaskInit初始化函数中，调用virtualMotorInit函数，初始化枚举中的所有电机
``` c
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
```

4. 在virtualMotorTaskInit初始化函数中，给virtualMotorTaskStructure.vmCan2SendFun/virtualMotorTaskStructure.vmCan1SendFun两个函数指针赋值，这两个函数的作用是将一个现有的can报文发送到对应的can1/can2总线上去。
``` c
	virtualMotorTaskStructure.vmCan2SendFun = vmCan2SendFun;	//发送can2信息的函数指针赋值
	virtualMotorTaskStructure.vmCan1SendFun = vmCan1SendFun;	//发送can1信息的函数指针赋值
	
	//上述两个函数实现方法如下，可以根据自己程序的不同修改此实现方法，只要能够保证msg能够最终发送到can总线上即可
	void vmCan2SendFun(CanTxMsg msg)
	{
		while(CAN_Transmit(CAN1, &msg) == CAN_TxStatus_NoMailBox);
	}

	void vmCan1SendFun(CanTxMsg msg)
	{
		while(CAN_Transmit(CAN2, &msg) == CAN_TxStatus_NoMailBox);
	}
```

5. 设置宏VM_FEEDBACK_HZ，为所有电机反馈的频率，不要太高，容易是can发送邮箱溢出。
``` c
 #define VM_FEEDBACK_HZ 100	//电机反馈频率，最好能被1000整除，实际反馈略小
```

6. 将vmCtrlMsgRecieve函数放置到can1 can2总线的中断里
``` c
void CAN1_RX0_IRQHandler(void)
{
	...
	CanRxMsg rx_message;	
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		...
		vmCtrlMsgRecieve(&rx_message, 0);
		...
	}
	...
}

void CAN2_RX0_IRQHandler(void)
{
	...
	CanRxMsg rx_message;	
    if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		...
		vmCtrlMsgRecieve(&rx_message, 1);
		...
	}
	...
}
```

7. 由于个人能力有限，目前此程序无法模拟出控制报文中转矩电流实际控制情况，所以通过设置宏VM_CONTROL_TYPE，重新定义控制报文中控制信息的意义
	* 如果设置为VM_RPM，则代表控制报文(0x1ff 0x200 0x2ff)中的控制信息控制的是转速，单位是rpm
	* 如果设置为VM_ACC，则代表控制报文(0x1ff 0x200 0x2ff)中的控制信息控制的是加速度，单位是rpm/s
``` c
#define VM_CONTROL_TYPE VM_RPM 
```	

8. 将函数void virtualMotorTask(void *pvParameters)作为一个FreeRTOS的一个任务
``` c
#define VISUAL_MOTOR_TASK_PRIO 11
#define VISUAL_MOTOR_STK_SIZE 256
static TaskHandle_t Visual_Motor_Handler;

xTaskCreate((TaskFunction_t)virtualMotorTask,          //任务函数
			(const char *)"virtualMotor_task",          //任务名称
			(uint16_t)VISUAL_MOTOR_STK_SIZE,            //任务堆栈大小
			(void *)NULL,                        //传递给任务函数的参数
			(UBaseType_t)VISUAL_MOTOR_TASK_PRIO,        //任务优先级
			(TaskHandle_t *)&Visual_Motor_Handler); //任务句柄
```
9. 初始化can总线时，把can的模式设置成回环模式

	
## 可能存在的问题
1. 如果模拟多个电机的话，因为需要频繁进行can发送，可能会导致can发送fifo的溢出，fifo溢出时，调用库函数进行发送是无效的。因此，最好是在发送时，判断一下邮箱是否已经满了，如果满了就一直发送。对应标准库的处理方法如下：
``` c
	while(CAN_Transmit(CAN1, &SendCanTxMsg) == CAN_TxStatus_NoMailBox);
```

## 目前程序的不足
1. 不能实际反映出转矩电流的实际控制效果，因此需要修改控制报文数据的意义，可以表示为期望电机达到的转速，或者是期望达到的加速度。
2. 无法加入物理模型，实现多个电机之间的相互作用以及机械方面的干涉。

联系qq 2296054658
