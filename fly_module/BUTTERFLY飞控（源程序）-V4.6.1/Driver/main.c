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
//����Ƽ���Ʒ
//�з��Ŷӣ����������Ŷ�
//��ϵ��ʽ��QQȺ��471023785
            ���䣺qitiansizhou@163.com
            �Ա���https://shop128265493.taobao.com/
//�������˳���ֻ��������ѧϰ��ѧ����ֹ������ҵ��;���������ؾ���
//�汾��V4.5
//���ڣ�20160112
//�޸�˵����
//
1��������ע��
2���Ż��˽ṹ
3��ȥ���˶���ĳ�����룬�������
4, ����Ӳ�����Ծ�ƫ����
*/
extern W_AND_ANGLE   w_and_angle;	//��sysconfig.h�еĶ������Ӧ�����ⲿ����
extern u16 Speed_FR;				 //λ��Control_200Hz.h�����е��ٶȿ��Ʊ���
extern __IO uint16_t ADCConvertedValue;
extern uint8_t Four_Axis_UNLOCK;     //һ������ ��ɻ���ֹͣ
static uint32_t MPU_counter = 0;
uint8_t  system_launch_succeed = 0;
extern float Desire_angle_yaw_flag;
extern volatile uint8_t EN_I_Flag;
extern int8_t MPU6050_First;

void NVIC_Configuration(void);
void System_State_Check(void);
void My_System_Init(void);

void INSPAIRE()//������������while(1)
{
	
	while(1) //10msѭ��һ��
	{
		LEDALL_ON;
		//���������Ϣ���ֱ��Ǻ����������ƫ���Ƕ�
		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		delay_Ms_Loop(2);
		MPU_counter ++;
		if(MPU_counter%20 == 0) //����ϵͳ��ѹ���
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
	Four_Axis_UNLOCK  = 0;//һ������ ��ɻ���ֹͣ
	MPU6050_First = 10;
	Speed_FR = 0;
	My_System_Init(); //���ڸ�������ĳ�ʼ��
	Desire_angle_yaw_flag = w_and_angle.angle_yaw;
	//������ⲿ�жϻ�Ӱ��ʹ��systickд���ӳٺ���������STM32��Ӳ��ȱ�ݣ�Ŀǰû���ҵ�����������滻������ʹ��delay_Ms_Loop
	MPU6050_Interrupt_Init();
	delay_Ms_Loop(1000);
	//�˴��ǳ�ȥMPU6050�ϵ�֮�󼸷��ӵ�yaw�ǶȵĲ���
	yaw_pre = 0; yaw_now = 0;
	LEDALL_OFF;
	while(1)
	{
		delay_Ms_Loop(3000);
		LEDALL_ON;
		delay_Ms_Loop(3000);
		LEDALL_OFF;
		//���������Ϣ���ֱ��Ǻ����������ƫ���Ƕ�
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
	NVIC_InitTypeDef    NVIC_InitStructure; 					   //�������������жϵĽṹ�����

/* #ifdef...#else...#endif�ṹ�������Ǹ���Ԥ�������������ж���������ʼ��ַ*/   
#ifdef  VECT_TAB_RAM  
  		/* �ж���������ʼ��ַ�� 0x20000000 ��ʼ */ 
 		NVIC_SetVectorTable(NVIC_VectTab_RAM , 0x0); 
#else 	/* VECT_TAB_FLASH */
  		/* �ж���������ʼ��ַ�� 0x80000000 ��ʼ */ 
  		NVIC_SetVectorTable(NVIC_VectTab_FLASH , 0x0);   
#endif
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�ж����ȼ�����  ��ռʽ���ȼ�������Ϊ2λ����Ӧ���ȼ�ռ2λ
	
	/* ���� TIM3 �ж�, 0����ռ���ȼ���0����ռ���ȼ���ϵͳ�������� */	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	/* �����ⲿ�ж�, 2����ռ���ȼ���2����ռ���ȼ���nRF�źŽ����ж� */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* �����ⲿ�ж�, 0����ռ���ȼ���0����ռ���ȼ���6050���ݽ����ж� */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//�������ж�
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
	delay_init(72);	   	 		//��ʱ��ʼ��	
	for(i=0;i<5;i++)
		delay_Ms_Loop(1000);	//�ϵ����ʱ	ȷ�������ȶ�
	
	LED_Init();	
	LEDALL_ON;
	timer2_init();					//PWM�����ʱ����ʼ��
	timer3_init();					//PWM�����ʱ����ʼ��
	IIC_Init();
	uart_init(38400); 	    //�����ô��ڳ�ʼ��
	My_usart2_init(38400);	//�����ô��ڳ�ʼ��
	printf("��ӭʹ������Ƽ�BUTTERFLY������\r\n");
	printf("QQȺ��471023785\r\n");
	LEDALL_OFF;
	MPU6050_Init();					//6050��ʼ��
	SPI1_INIT();						//SPI��ʼ��������nRFģ��
	flag = NRF_CHECK();			//���NRFģ���Ƿ���������
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
	NRF24L01_INIT();						//nRF��ʼ��
	SetRX_Mode();								//����Ϊ����ģʽ
	NRF24L01_INIT();						//nRF��ʼ��
	NRF_GPIO_Interrupt_Init();	//nRFʹ�õ��ⲿ�жϵ����ų�ʼ��
	tim4_init();								//��ʱ�жϣ���Ϊϵͳ�Ŀ���Ƶ��	
	adcInit();									//ADC��ʼ����������ص�ѹ
}

void System_State_Check(void)
{
	float Battery_Vol = 0;	

	Battery_Vol = (float)ADCConvertedValue;
	Battery_Vol = ((Battery_Vol*3.3f*2.0f*1.1842*0.93)/4096.0f);
	//���֮ǰ���Ϊ3.8�����֮��Ϊ3.2
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



