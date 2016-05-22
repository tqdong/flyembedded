/* Includes ------------------------------------------------------------------*/
//mian.h�������������������Ҫ��ͷ�ļ�
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
//����Ƽ���Ʒ
//�з��Ŷӣ����������Ŷ�
//��ϵ��ʽ��QQȺ��471023785
            ���䣺qitiansizhou@163.com
            �Ա���http://shop125061094.taobao.com/
						����: http://qitiantech.cn/
//�������˳���ֻ��������ѧϰ��ѧ����ֹ������ҵ��;���������ؾ���
//�汾��V4.5
//���ڣ�20151027
//�޸�˵����
//
1���������ű�������
2�����뷢�ͼ�����
3�����뾲ƫӲ����������
*/
void My_System_Init(void);
//extern __IO uint16_t ADCConvertedValue;
extern uint16_t ADC_Result;
extern uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];
void Inspaire()//������������while(1)
{
	static uint32_t NRF_Counter = 0;
	static uint32_t NRF_Check_Sum = 0;
	float Power_Check = 0.0;
	uint8_t i;
	uint32_t RUN_Times = 0;
	while(1)
	{
		//����ɨ��ң�صĸ���ҡ�˵�λ�ã�Ȼ����NRF����ȥ
		//��ͷ�Լ����͵Ĵ���0-99ѭ������
		NRF24L01_TXDATA[0] = 0XA5;
		NRF_Counter ++;
		NRF24L01_TXDATA[1] = NRF_Counter;
		if(NRF_Counter >= 99)
			NRF_Counter = 0;
		
		//�������
		adcInit1();
		ADC_Result = ADC_Result/4;
		NRF24L01_TXDATA[2] = (uint8_t)(ADC_Result&0x00ff);
		NRF24L01_TXDATA[3] = (uint8_t)((ADC_Result>>8)&0x00ff);

		//���YAW
		adcInit0();
		ADC_Result = ADC_Result/4;
		NRF24L01_TXDATA[4] = (uint8_t)(ADC_Result&0x00ff);
		NRF24L01_TXDATA[5] = (uint8_t)((ADC_Result>>8)&0x00ff);
		
		//���Pitch
		adcInit4();
		ADC_Result = 4096 - ADC_Result;
		ADC_Result = ADC_Result/4;
		NRF24L01_TXDATA[6] = (uint8_t)(ADC_Result&0x00ff);
		NRF24L01_TXDATA[7] = (uint8_t)((ADC_Result>>8)&0x00ff);
		
		//���Roll
		adcInit6();
		ADC_Result = ADC_Result/4;
		NRF24L01_TXDATA[8] = (uint8_t)(ADC_Result&0x00ff);
		NRF24L01_TXDATA[9] = (uint8_t)((ADC_Result>>8)&0x00ff);
		
		//��ⰴ��
		if(KEY1_ON)
			NRF24L01_TXDATA[10] = 1;
		else if(KEY2_ON)
			NRF24L01_TXDATA[10] = 2;	
		else
			NRF24L01_TXDATA[10] = 0;
		
		//������ǰ���ʼ��ƫ
		adcInit3();
		ADC_Result = ADC_Result/16;
		NRF24L01_TXDATA[20] = (uint8_t)(ADC_Result&0x00ff);
		
		//���������ҳ�ʼ��ƫ
		adcInit2();
		ADC_Result = ADC_Result/16;
		NRF24L01_TXDATA[21] = (uint8_t)(ADC_Result&0x00ff);
		
		//������
		adcInit5();
		Power_Check = ADC_Result*3.3*2/4096.0;
		if(Power_Check < 3.7)
		{
			//��������ʾ��������
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
		
		//���к�У�鴦��
		NRF_Check_Sum = 0;
		for(i=0;i<30;i++)
		{
				NRF_Check_Sum = NRF_Check_Sum + NRF24L01_TXDATA[i];
		}
		NRF24L01_TXDATA[30] = (uint8_t)(NRF_Check_Sum&0x00ff);
		NRF24L01_TXDATA[31] = (uint8_t)((NRF_Check_Sum>>8)&0x00ff);
		delay_Ms_Loop(5);
		//����һ������ÿ�뷢50-100�����ݰ�
		//delay_Ms_Loop(10);
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6));
		NRF_IRQ();
		NRF_TxPacket(NRF24L01_TXDATA,32);
	}
}

int main(void)
{

	My_System_Init(); //���ڸ�������ĳ�ʼ��
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
	flag = NRF_CHECK();//���NRFģ���Ƿ���������
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
	NRF24L01_INIT();	//nRF24L01��ʼ��
//	NRF24L01_INIT();	//nRF24L01��ʼ��
	NRF_GPIO_Interrupt_Init();	//
	//��ֹ�������ͻȻ����
	while(1)
	{
				//�������
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
