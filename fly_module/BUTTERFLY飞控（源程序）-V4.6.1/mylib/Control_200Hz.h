#ifndef _Control_200Hz_H
#define _Control_200Hz_H


void moto_init(void);		//电调启动初始化
void tim4_init(void);
void my_tim4_IRQHandler(void);
void NRF_Data_Receive(void);
void PID_UPdata(void);

#endif
