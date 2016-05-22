
//计算PID的输出值
#include "PID.h"
#include "led.h"
#include "sysconfig.h"
#include "math.h"
#include "nrf24l01.h"
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
//PID参数宏定义
//*************************************

#define PID_Limit -0.1f
#define PID_I_Limit 400.0f
#define PID_W_Limit 2000.0f

////以下是75mm桨的参数 20150727
//P为72/30.1 D为36/30.1
#define rollP_IN  1.993f
#define rollI_IN  0.01f
#define rollD_IN  13.0f

#define rollP_EXC  -4.0f
#define rollI_EXC  -0.01f

#define pitchP_EXC  -4.0f
#define pitchI_EXC  -0.01f

#define pitchP_IN 1.8f					   
#define pitchI_IN 0.01f 
#define pitchD_IN 13.0f


#define w_yawP   25.0f	//注意：yaw轴的是角速度闭环，而不是角度闭环												   
#define w_yawI   0
#define w_yawD   0

#define yawP   11.14												   
#define yawI   0.04
#define yawD   7.9




//*************************************

volatile float w_P_yaw = 0.0, w_D_yaw = 0.0;
volatile float Desire_angle_yaw_flag = 3.0;
extern float Desire_angle_yaw; 
volatile float angle_P_roll = 0.0, angle_P_pitch = 0.0, angle_P_yaw = 0.0, angle_D_roll = 0.0, angle_D_pitch = 0.0, angle_D_yaw = 0.0;
volatile float angle_I_roll = 0.0, angle_I_pitch = 0.0, angle_I_yaw = 0.0;
volatile float angle_I_roll_pre = 0, angle_I_pitch_pre = 0;
volatile float PID_roll_out = 0.0, PID_pitch_out = 0.0, PID_yaw_out = 0.0;
volatile float PID_roll_out_pre = 0, PID_pitch_out_pre = 0, PID_yaw_out_pre = 0;
volatile float pitch_int = 0.0, roll_int = 0.0;
volatile float Desire_angle_roll_SUM = 0,Desire_angle_pitch_SUM = 0;
volatile float Desire_W_roll = 0,Desire_W_pitch = 0;
volatile float Error_Angle_roll = 0,Error_Angle_pitch = 0;
volatile float Error_W_roll = 0,Error_W_pitch = 0;
volatile float Now_W_roll_Err = 0,Now_W_pitch_Err = 0;
volatile float Pre_W_roll_Err = 0,Pre_W_pitch_Err = 0;
volatile float Error_W_roll_I = 0,Error_W_pitch_I = 0;
volatile float Error_Angle_roll_I = 0,Error_Angle_pitch_I = 0;
extern W_AND_ANGLE w_and_angle;	//和sysconfig中的定义相对应，是外部变量
								//此变量存在于MPU6050.c中
extern uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据									
extern float Desire_angle_roll,Desire_angle_pitch,Desire_angle_yaw;
extern uint8_t Four_Axis_UNLOCK;     //一键解锁 起飞或者停止
extern float Desire_w_yaw;
extern 	float GPS_Lo_Pitch_out,GPS_Lo_Roll_out;
volatile float GPS_Lo_Pitch_out_Pre_Now,GPS_Lo_Roll_out_Pre_Now;
volatile float w_roll_pre = 0.0,w_pitch_pre = 0.0,w_yaw_pre = 0.0;
//extern float HIGH_Ultrasonic;
extern uint8_t  system_launch_succeed;
volatile uint8_t EN_I_Flag = 0;
extern uint8_t GPS_OUT_Isvalid;
extern volatile int BT_Throttle, BT_Yaw, BT_Pitch, BT_Roll;
void PID_calculate(void)	
{
	float now,d;

	Desire_angle_roll_SUM = Desire_angle_roll;
	Desire_angle_pitch_SUM = Desire_angle_pitch;
	//对作用飞机上的偏角进行限制
	if(Desire_angle_roll_SUM > 40)
	{
		Desire_angle_roll_SUM = 40;
	}
	else if(Desire_angle_roll_SUM < -40)
	{
		Desire_angle_roll_SUM = -40;
	}
	
	if(Desire_angle_pitch_SUM > 40)
	{
		Desire_angle_pitch_SUM = 40;
	}
	else if(Desire_angle_pitch_SUM < -40)
	{
		Desire_angle_pitch_SUM = -40;
	}
	//当前角度误差
	Error_Angle_roll = (Desire_angle_roll_SUM - w_and_angle.angle_roll);
	Error_Angle_pitch = (Desire_angle_pitch_SUM - w_and_angle.angle_pitch);
	
	Error_Angle_roll_I += Error_Angle_roll;
	Error_Angle_pitch_I += Error_Angle_pitch;
	
	//积分限幅
	if( Error_Angle_roll_I > 500 )
	  Error_Angle_roll_I = 500;
	if( Error_Angle_roll_I < -500 )
	  Error_Angle_roll_I = -500;
	if( Error_Angle_pitch_I > 500 )
	  Error_Angle_pitch_I = 500;
	if( Error_Angle_pitch_I < -500 )
	  Error_Angle_pitch_I = -500;
	
	//外环的P 和 I产生期望角速度
	Desire_W_roll  = (Error_Angle_roll)*rollP_EXC + Error_Angle_roll_I*rollI_EXC;
	Desire_W_pitch = (Error_Angle_pitch)*pitchP_EXC + Error_Angle_pitch_I*pitchI_EXC;
	
	//由外环的P和I得到的输出减去此时的角速度，从而得到角速度误差
	Error_W_roll = Desire_W_roll - w_and_angle.w_roll;
	Error_W_pitch = Desire_W_pitch - w_and_angle.w_pitch;
	
	Error_W_roll_I += Error_W_roll;
	Error_W_pitch_I += Error_W_pitch;
	
	//积分限幅
	if( Error_W_roll_I > 500 )
	  Error_W_roll_I = 500;
	if( Error_W_roll_I < -500 )
	  Error_W_roll_I = -500;
	if( Error_W_pitch_I > 500 )
	  Error_W_pitch_I = 500;
	if( Error_W_pitch_I < -500 )
	  Error_W_pitch_I = -500;
	
	Now_W_roll_Err = Error_W_roll;
	Now_W_pitch_Err = Error_W_pitch;
	
	PID_roll_out = -Error_W_roll*rollP_IN + Error_W_roll_I*rollI_IN - (Now_W_roll_Err - Pre_W_roll_Err)*rollD_IN;
	PID_pitch_out = -Error_W_pitch*pitchP_IN + Error_W_pitch_I*pitchI_IN - (Now_W_pitch_Err - Pre_W_pitch_Err)*pitchD_IN;
	
	//更新角速度误差
	Pre_W_roll_Err = Now_W_roll_Err;
	Pre_W_pitch_Err = Now_W_pitch_Err;

	
	//对yaw角度的处理，yaw是一个角速度和角度的闭环稳定系统
	if((fabs(Desire_w_yaw) < 4.0))
	{
		w_P_yaw = 0;
		w_D_yaw = 0;
		Desire_angle_yaw = Desire_angle_yaw_flag;
		
		now = w_and_angle.angle_yaw;
		d = Desire_angle_yaw;
	
		if(d >= 180)
		{
			if((now - d) < 0)
			{
				if( ((d - now)<180)) // && ((now - d)<0) 
				{// turn reserve
					 angle_P_yaw   = -(d - now) * yawP;
					 angle_I_yaw += -(d - now) * yawI;
				}
				else
				{// turn clockwise
					 angle_P_yaw   = (now + (360 - d)) * yawP;
					 angle_I_yaw += (now + (360 - d)) * yawI;
				}
			}
			else
			{// turn clockwise
					angle_P_yaw   = (now - d) * yawP;
					angle_I_yaw += (now - d) * yawI;
			}
		}
		else
		{
			if((now - d) >= 0)
			{
				if( ((now - d)<180)) // && ((now - d)>0) 
				{// turn clockwise
					 angle_P_yaw   = (now - d) * yawP;
					 angle_I_yaw += (now - d) * yawI; 
				}
				else
				{// turn reserve
					 angle_P_yaw   = -(d+ (360 - now)) * yawP;
					 angle_I_yaw += -(d+ (360 - now)) * yawI; 
				}
			}
			else
			{// turn reserve
					angle_P_yaw   = -(d - now) * yawP;
					angle_I_yaw += -(d - now) * yawI;
			}
		}
		if(angle_I_yaw > 300)
		{
			angle_I_yaw = 300;
		}
		else if(angle_I_yaw < -300)
		{
			angle_I_yaw = -300;
		}
		angle_D_yaw = w_and_angle.w_yaw * yawD;
	}
	else
	{
//		angle_P_yaw = 0;
//		angle_D_yaw = 0;
		
		Desire_angle_yaw_flag = w_and_angle.angle_yaw;
		w_P_yaw   = (Desire_w_yaw - w_and_angle.w_yaw) * w_yawP;	//量程上下各是500，/25后约是20，代表满舵角速度是20度/s	
		w_D_yaw   = (w_and_angle.w_yaw - w_yaw_pre) * w_yawD;	
		w_yaw_pre = w_and_angle.w_yaw;	
	}	
	//此时使能I
	if( (((uint16_t)NRF24L01_RXDATA[3]*256 + (uint16_t)NRF24L01_RXDATA[2]) > 500) || (BT_Throttle > 500) )
	{
		EN_I_Flag = 1;
	}
	//初始时关闭I，防止震荡
	if( (((uint16_t)NRF24L01_RXDATA[3]*256 + (uint16_t)NRF24L01_RXDATA[2]) < 100) && (BT_Throttle < 100) )
	{
		Desire_angle_yaw_flag = w_and_angle.angle_yaw;
		EN_I_Flag = 0;
	}
	if(EN_I_Flag == 0)
	{
		angle_I_roll = 0;
		//防止电池放歪
		if(angle_I_pitch > 50)
			angle_I_pitch = 50;
		else if(angle_I_pitch < -50)
			angle_I_pitch = -50;
		//angle_I_pitch = 0;
		angle_I_yaw = 0;
	}

	PID_yaw_out   = -(angle_P_yaw + angle_D_yaw + angle_I_yaw) + w_P_yaw + w_D_yaw;	
	
	if(!Four_Axis_UNLOCK)
	{
		Desire_angle_yaw_flag = w_and_angle.angle_yaw;
		PID_yaw_out = 0;
		PID_pitch_out = 0;
		PID_roll_out = 0;
		//EN_I_Flag = 1;
		angle_I_yaw = 0;
	}
}






