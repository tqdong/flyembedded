#include "stm32f10x.h"
#include "sysconfig.h"
#include "system_stm32f10x.h"
#include "PID.h"
#include "PWM_output.h"
#include "Control_200Hz.h"
#include "MPU6050.h"
#include "delay.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
//#include "IMU.h"
#include "spi.h"
#include "nrf24l01.h"
#include "myiic.h"
#include "led.h"
#include "math.h"
#include "drv_adc.h"
//#include "MS5611.h"
#include "usart2.h"
/*
                  ////////////

            /////////////////////////                   /////////////////////////////
           ////                /////                               //////
          ////                /////                               //////
         /////               /////                  /////////////////////////////////////////
        //////////////////////////                             //////  /////
       /////                                                  //////     /////
     /////    ///////////////////                            //////        /////
    ////      ////          /////                           /////            //////
   ////       ////          /////                          /////              ///////
  ////        ////          /////                         /////                ////////
 /////        ///////////////////                        /////                   /////////
//启天科技出品
//研发团队：启天四轴团队
//联系方式：QQ群：471023785
            邮箱：qitiansizhou@163.com
            淘宝：https://shop128265493.taobao.com/
//声明：此程序只可以用于学习教学，禁止用于商业用途，否则后果必究！
//版本：V4.5
//日期：20160112
//修改说明：
//
1，增加了注释
2，优化了结构
3，去除了多余的程序代码，方便理解
4, 加入硬件调试静偏功能
*/
extern W_AND_ANGLE   w_and_angle;	//和sysconfig.h中的定义相对应，是外部变量
extern u16 Speed_FR;				 //位于Control_200Hz.h函数中的速度控制变量
extern __IO uint16_t ADCConvertedValue;
extern uint8_t Four_Axis_UNLOCK;     //一键解锁 起飞或者停止
static uint32_t MPU_counter = 0;
uint8_t  system_launch_succeed = 0;
extern float Desire_angle_yaw_flag;
extern volatile uint8_t EN_I_Flag;
extern int8_t MPU6050_First;

void NVIC_Configuration(void);
void System_State_Check(void);
void My_System_Init(void);

void INSPAIRE()//这是主函数的while(1)
{
	
	while(1) //10ms循环一次
	{
		LEDALL_ON;
		//输出调试信息，分别是横滚，俯仰，偏航角度
		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		delay_Ms_Loop(2);
		MPU_counter ++;
		if(MPU_counter%20 == 0) //运行系统电压检测
		{				
			System_State_Check();
			if(MPU_counter >= 10000)
				MPU_counter = 0;
		}
	}// end of while(1)
}

int main(void)
{
//	int i;
	float yaw_pre = 0, yaw_now = 0;
	Four_Axis_UNLOCK  = 0;//一键解锁 起飞或者停止
	MPU6050_First = 10;
	Speed_FR = 0;
	My_System_Init(); //对于各种外设的初始化
	Desire_angle_yaw_flag = w_and_angle.angle_yaw;
	//下面的外部中断会影响使用systick写的延迟函数，这是STM32的硬件缺陷，目前没有找到解决方法。替换方法：使用delay_Ms_Loop
	MPU6050_Interrupt_Init();
	delay_Ms_Loop(1000);
	//此处是除去MPU6050上电之后几分钟的yaw角度的波动
	yaw_pre = 0; yaw_now = 0;
	LEDALL_OFF;
	while(1)
	{
		delay_Ms_Loop(3000);
		LEDALL_ON;
		delay_Ms_Loop(3000);
		LEDALL_OFF;
		//输出调试信息，分别是横滚，俯仰，偏航角度
		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);		
		
		yaw_now = w_and_angle.angle_yaw;
		if( (fabs(yaw_now - yaw_pre) > 0.9) || (yaw_now == 0) )
		{
		}
		else
		{
			break;
		}
		if(yaw_now == 0)
		{			
		}
		yaw_pre = yaw_now;
	}
		LED2_ON;
		Four_Axis_UNLOCK = 1;
		LEDALL_OFF;		
		//while(1);
		INSPAIRE();
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef    NVIC_InitStructure; 					   //定义用于配置中断的结构体变量

/* #ifdef...#else...#endif结构的作用是根据预编译条件决定中断向量表起始地址*/   
#ifdef  VECT_TAB_RAM  
  		/* 中断向量表起始地址从 0x20000000 开始 */ 
 		NVIC_SetVectorTable(NVIC_VectTab_RAM , 0x0); 
#else 	/* VECT_TAB_FLASH */
  		/* 中断向量表起始地址从 0x80000000 开始 */ 
  		NVIC_SetVectorTable(NVIC_VectTab_FLASH , 0x0);   
#endif
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组  抢占式优先级别设置为2位；响应优先级占2位
	
	/* 开启 TIM3 中断, 0级先占优先级，0级后占优先级，系统控制周期 */	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	/* 开启外部中断, 2级先占优先级，2级后占优先级，nRF信号接收中断 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 开启外部中断, 0级先占优先级，0级后占优先级，6050数据接收中断 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//蓝牙的中断
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void My_System_Init(void)
{
uint8_t flag, i=0;
	NVIC_Configuration();
	delay_init(72);	   	 		//延时初始化	
	for(i=0;i<5;i++)
		delay_Ms_Loop(1000);	//上电短延时	确保供电稳定
	
	LED_Init();	
	LEDALL_ON;
	timer2_init();					//PWM输出定时器初始化
	timer3_init();					//PWM输出定时器初始化
	IIC_Init();
	uart_init(38400); 	    //调试用串口初始化
	My_usart2_init(38400);	//蓝牙用串口初始化
	printf("欢迎使用启天科技BUTTERFLY四旋翼\r\n");
	printf("QQ群：471023785\r\n");
	LEDALL_OFF;
	MPU6050_Init();					//6050初始化
	SPI1_INIT();						//SPI初始化，用于nRF模块
	flag = NRF_CHECK();			//检查NRF模块是否正常工作
	if(flag != 1)
	{
		while(1)
		{
			LEDALL_OFF;
			delay_Ms_Loop(200);
			LEDALL_ON;
			delay_Ms_Loop(200);
		}
	}
	NRF24L01_INIT();						//nRF初始化
	SetRX_Mode();								//设置为接收模式
	NRF24L01_INIT();						//nRF初始化
	NRF_GPIO_Interrupt_Init();	//nRF使用的外部中断的引脚初始化
	tim4_init();								//定时中断，作为系统的控制频率	
	adcInit();									//ADC初始化，测量电池电压
}

void System_State_Check(void)
{
	float Battery_Vol = 0;	

	Battery_Vol = (float)ADCConvertedValue;
	Battery_Vol = ((Battery_Vol*3.3f*2.0f*1.1842*0.93)/4096.0f);
	//起飞之前检测为3.8，起飞之后为3.2
	if(EN_I_Flag)
	{
		if(Battery_Vol < 3.8)
		{
				LEDALL_FLASH(1,500);    
		}
		else
		{
		}
	}
	else
	{
		if(Battery_Vol < 3.2)
		{
				//LEDALL_FLASH(1,500);    
		}
		else
		{
		}
	}
}



