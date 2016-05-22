/* Includes ------------------------------------------------------------------*/
//mian.h包含了我们这个工程需要的头文件
#include "main.h"
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
            淘宝：http://shop125061094.taobao.com/
						官网: http://qitiantech.cn/
//声明：此程序只可以用于学习教学，禁止用于商业用途，否则后果必究！
//版本：V4.5
//日期：20151027
//修改说明：
//
1，加入油门保护机制
2，加入发送检测机制
3，加入静偏硬件调整机制
*/
void My_System_Init(void);
//extern __IO uint16_t ADCConvertedValue;
extern uint16_t ADC_Result;
extern uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];
void Inspaire()//这是主函数的while(1)
{
	static uint32_t NRF_Counter = 0;
	static uint32_t NRF_Check_Sum = 0;
	float Power_Check = 0.0;
	uint8_t i;
	uint32_t RUN_Times = 0;
	while(1)
	{
		//依次扫描遥控的各个摇杆的位置，然后用NRF发出去
		//包头以及发送的次数0-99循环发送
		NRF24L01_TXDATA[0] = 0XA5;
		NRF_Counter ++;
		NRF24L01_TXDATA[1] = NRF_Counter;
		if(NRF_Counter >= 99)
			NRF_Counter = 0;
		
		//检测油门
		adcInit1();
		ADC_Result = ADC_Result/4;
		NRF24L01_TXDATA[2] = (uint8_t)(ADC_Result&0x00ff);
		NRF24L01_TXDATA[3] = (uint8_t)((ADC_Result>>8)&0x00ff);

		//检测YAW
		adcInit0();
		ADC_Result = ADC_Result/4;
		NRF24L01_TXDATA[4] = (uint8_t)(ADC_Result&0x00ff);
		NRF24L01_TXDATA[5] = (uint8_t)((ADC_Result>>8)&0x00ff);
		
		//检测Pitch
		adcInit4();
		ADC_Result = 4096 - ADC_Result;
		ADC_Result = ADC_Result/4;
		NRF24L01_TXDATA[6] = (uint8_t)(ADC_Result&0x00ff);
		NRF24L01_TXDATA[7] = (uint8_t)((ADC_Result>>8)&0x00ff);
		
		//检测Roll
		adcInit6();
		ADC_Result = ADC_Result/4;
		NRF24L01_TXDATA[8] = (uint8_t)(ADC_Result&0x00ff);
		NRF24L01_TXDATA[9] = (uint8_t)((ADC_Result>>8)&0x00ff);
		
		//检测按键
		if(KEY1_ON)
			NRF24L01_TXDATA[10] = 1;
		else if(KEY2_ON)
			NRF24L01_TXDATA[10] = 2;	
		else
			NRF24L01_TXDATA[10] = 0;
		
		//检测参数前后初始静偏
		adcInit3();
		ADC_Result = ADC_Result/16;
		NRF24L01_TXDATA[20] = (uint8_t)(ADC_Result&0x00ff);
		
		//检测参数左右初始静偏
		adcInit2();
		ADC_Result = ADC_Result/16;
		NRF24L01_TXDATA[21] = (uint8_t)(ADC_Result&0x00ff);
		
		//检测电量
		adcInit5();
		Power_Check = ADC_Result*3.3*2/4096.0;
		if(Power_Check < 3.7)
		{
			//灯闪，提示电量不足
			if(RUN_Times%12 == 0)
			{
				LEDALL_OFF;
			}
			else if(RUN_Times%22 == 0)
			{
				LEDALL_ON;
			}
			RUN_Times ++;
			if(RUN_Times > 1000000)
			{
				RUN_Times = 0;
			}
		}
		else
		{
			LEDALL_ON;
		}
		
		//进行和校验处理
		NRF_Check_Sum = 0;
		for(i=0;i<30;i++)
		{
				NRF_Check_Sum = NRF_Check_Sum + NRF24L01_TXDATA[i];
		}
		NRF24L01_TXDATA[30] = (uint8_t)(NRF_Check_Sum&0x00ff);
		NRF24L01_TXDATA[31] = (uint8_t)((NRF_Check_Sum>>8)&0x00ff);
		delay_Ms_Loop(5);
		//发送一个包，每秒发50-100个数据包
		//delay_Ms_Loop(10);
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6));
		NRF_IRQ();
		NRF_TxPacket(NRF24L01_TXDATA,32);
	}
}

int main(void)
{

	My_System_Init(); //对于各种外设的初始化
	Inspaire();
}

void NVIC_Configuration(void)
{
}

void My_System_Init(void)
{
	uint8_t flag;
	LED_INIT();
	Key_INIT();
	LED1_OFF;
	LED2_OFF;
	//while(1);
	delay_Ms_Loop(1000);
	NVIC_Configuration();
	SPI1_INIT();
	flag = NRF_CHECK();//检查NRF模块是否正常工作
	if(flag != 1)
	{
		while(1)
		{
			LED1_OFF;
			delay_Ms_Loop(200);
			LED1_ON;
			delay_Ms_Loop(200);
		}
	}
	NRF24L01_INIT();	//nRF24L01初始化
//	NRF24L01_INIT();	//nRF24L01初始化
	NRF_GPIO_Interrupt_Init();	//
	//防止开机后的突然启动
	while(1)
	{
				//检测油门
		adcInit1();
		ADC_Result = ADC_Result/4;
		if(ADC_Result < 200)
			break;
	}
	LEDALL_ON;
}
void System_STate_Check(void)
{
		//float Battery_Vol = 0;
}
