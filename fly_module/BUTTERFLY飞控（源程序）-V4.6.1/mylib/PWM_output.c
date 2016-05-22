
//对timer2定时器进行操作，利用PID的输出改变比较寄存器CCR的值，从而改变PWM输出高电平时间
#include "stm32f10x_tim.h"
#include "PWM_output.h"
//#include "myiic.h" 	//这个头文件包含了PAout(n)的定义与声明
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
//版本：V4.0
//日期：20151015
//修改说明：
//
*/
volatile uint16_t CCR1_Val = 0;						/* 初始化输出比较通道1计数周期变量 */
volatile uint16_t CCR2_Val = 0;						/* 初始化输出比较通道2计数周期变量 */
volatile uint16_t CCR3_Val = 1000;				   		/* 初始化输出比较通道3计数周期变量 */
volatile uint16_t CCR4_Val = 1000;						/* 初始化输出比较通道4计数周期变量 */


//此为PWM的输出定时器端口
void timer2_init(void)
{
 	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE); /* 打开 TIM2 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* 设置 GPIOA 上的 TIM2 1，2通道对应引脚 PA.0,PA.1为第二功能推挽输出 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 	设置timer2
	*  	计数重载值为1000
	*  	预分频值为3
	*  	时钟分割0
	*  	向上计数模式
	*	则产生的PWM信号频率为24KHz，占空比为CCRx_Val/1000
	*/
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = 2;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM2 , &TIM_TimeBaseStructure);		
	/* 	设置timer2的 OC1,OC2通道
	*  	工作模式为 PWM 输出模式
	*  	使能比较匹配输出极性
	*  	时钟分割0
	*  	向上计数模式
	*	设置各匹配值分别为 CCR1_Val, CCR2_Val
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	
	/* 使能预装载寄存器 */
	TIM_OC1PreloadConfig(TIM2 , TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
		
	/* 启动 TIM 计数 */
	TIM_Cmd(TIM2 , ENABLE);		
}
void timer3_init(void)
{
 	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); /* 打开 TIM2 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* 设置 GPIOA 上的 TIM3  3，4通道对应引脚 PB.0,PB.1为第二功能推挽输出 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 	设置timer3
	*  	计数重载值为1000
	*  	预分频值为3
	*  	时钟分割0
	*  	向上计数模式
	*	则产生的PWM信号频率为24KHz，占空比为CCRx_Val/1000
	*/
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseStructure.TIM_Prescaler = 2;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM3 , &TIM_TimeBaseStructure);		
	/* 	设置timer3的OC3,OC4 通道
	*  	工作模式为 PWM 输出模式
	*  	使能比较匹配输出极性
	*  	时钟分割0
	*  	向上计数模式
	*	设置各匹配值分别为 CCR3_Val, CCR4_Val
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	/* 使能预装载寄存器 */
	TIM_OC3PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
		
	/* 启动 TIM 计数 */
	TIM_Cmd(TIM3 , ENABLE);		
}

