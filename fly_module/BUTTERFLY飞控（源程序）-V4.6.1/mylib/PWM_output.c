
//��timer2��ʱ�����в���������PID������ı�ȽϼĴ���CCR��ֵ���Ӷ��ı�PWM����ߵ�ƽʱ��
#include "stm32f10x_tim.h"
#include "PWM_output.h"
//#include "myiic.h" 	//���ͷ�ļ�������PAout(n)�Ķ���������
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
//�汾��V4.0
//���ڣ�20151015
//�޸�˵����
//
*/
volatile uint16_t CCR1_Val = 0;						/* ��ʼ������Ƚ�ͨ��1�������ڱ��� */
volatile uint16_t CCR2_Val = 0;						/* ��ʼ������Ƚ�ͨ��2�������ڱ��� */
volatile uint16_t CCR3_Val = 1000;				   		/* ��ʼ������Ƚ�ͨ��3�������ڱ��� */
volatile uint16_t CCR4_Val = 1000;						/* ��ʼ������Ƚ�ͨ��4�������ڱ��� */


//��ΪPWM�������ʱ���˿�
void timer2_init(void)
{
 	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE); /* �� TIM2 ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* ���� GPIOA �ϵ� TIM2 1��2ͨ����Ӧ���� PA.0,PA.1Ϊ�ڶ������������ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 	����timer2
	*  	��������ֵΪ1000
	*  	Ԥ��ƵֵΪ3
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*	�������PWM�ź�Ƶ��Ϊ24KHz��ռ�ձ�ΪCCRx_Val/1000
	*/
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = 2;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM2 , &TIM_TimeBaseStructure);		
	/* 	����timer2�� OC1,OC2ͨ��
	*  	����ģʽΪ PWM ���ģʽ
	*  	ʹ�ܱȽ�ƥ���������
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*	���ø�ƥ��ֵ�ֱ�Ϊ CCR1_Val, CCR2_Val
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	
	/* ʹ��Ԥװ�ؼĴ��� */
	TIM_OC1PreloadConfig(TIM2 , TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
		
	/* ���� TIM ���� */
	TIM_Cmd(TIM2 , ENABLE);		
}
void timer3_init(void)
{
 	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); /* �� TIM2 ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* ���� GPIOA �ϵ� TIM3  3��4ͨ����Ӧ���� PB.0,PB.1Ϊ�ڶ������������ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 	����timer3
	*  	��������ֵΪ1000
	*  	Ԥ��ƵֵΪ3
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*	�������PWM�ź�Ƶ��Ϊ24KHz��ռ�ձ�ΪCCRx_Val/1000
	*/
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = 2;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM3 , &TIM_TimeBaseStructure);		
	/* 	����timer3��OC3,OC4 ͨ��
	*  	����ģʽΪ PWM ���ģʽ
	*  	ʹ�ܱȽ�ƥ���������
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*	���ø�ƥ��ֵ�ֱ�Ϊ CCR3_Val, CCR4_Val
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	/* ʹ��Ԥװ�ؼĴ��� */
	TIM_OC3PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
		
	/* ���� TIM ���� */
	TIM_Cmd(TIM3 , ENABLE);		
}

