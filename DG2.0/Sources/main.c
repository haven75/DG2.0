#include "includes.h"
unsigned int Flag=0,wait=9;
signed int steer=0,delay_count=0,StartDelay=0;
#define StartDelaySec 300


void FastSpeedMode();
void MiddleSpeedMode();
void SlowSpeedMode();
void OpenLoopMode();

void main(void)
 {
	initALL();
	while(wait>0);
	if(switch4==0)
		Set_Middlepoint();
	if(switch1==0&&switch2==0)
		OpenLoopMode();
	if(switch1==1&&switch2==1)
		FastSpeedMode();
	if(switch2==0&&switch1==1)
		MiddleSpeedMode();
	if(switch2==1&&switch1==0)
		SlowSpeedMode();
 }
void Pit0ISR()     
{
	Flag=1;
	frequency_measure();
	Get_speed();
	if(wait>0)
		wait--;
	if(1)
	{
		if(delay_count<500)
			delay_count++;    //·ÀÖ¹¿ª»ú´¥·¢
		else Ramp_Detect();
		if(Ramp_Flag==1)
			Ramp_Time++;
		if(Ramp_Time>40)
		{
			Up_Flag=2;
			Ramp_Flag=0;
		}
	}
	if(Up_Flag==2&&Ramp_Time_Delay>0)
		Ramp_Time_Delay--;
	//if(StartFlag==1&&RunFlag<2005)
	//	RunFlag++;
	if(switch3==0)
		StartDelay++;
	if(StartDelay>StartDelaySec+1)
		StartDelay=StartDelaySec+1;
	count++;
	PIT.CH[0].TFLG.B.TIF = 1;
}

void FastSpeedMode()
{
	for (;;) 
	{
		speed1=380;
		speed5=225;
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-240)
				steer=STEER_HELM_CENTER-250;
			if(steer>=STEER_HELM_CENTER+240)
				steer=STEER_HELM_CENTER+250;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER+8;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>=StartDelaySec)
				SpeedSet();
			speed_control();
		}
		Flag=0;
		Senddata();
	}
}
                   
void MiddleSpeedMode()
{
	for (;;) 
	{	speed1=350;
		speed5=230;
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-240)
				steer=STEER_HELM_CENTER-250;
			if(steer>=STEER_HELM_CENTER+240)
				steer=STEER_HELM_CENTER+250;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER+10;
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>=StartDelaySec)
				SpeedSet();
			speed_control();
		}
		Flag=0;
		Senddata();
	}
}

void SlowSpeedMode()
{
	for (;;) 
	{
		kp1=8.3;	kd1=38;  
		kp2=4.1;	kd2=34;
		kp3=3;		kd3=32;
		kp4=2.4;	kd4=30;
		
		speed_kp=0.9;
		speed_ki=0.31;
		speed_kd=0.25;
		speed1=320;
		speed2=290;
		speed3=270;
		speed4=250;
		speed5=235;
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-240)
				steer=STEER_HELM_CENTER-250;
			if(steer>=STEER_HELM_CENTER+240)
				steer=STEER_HELM_CENTER+250;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
			{
				steer=STEER_HELM_CENTER+13;
			}
			SET_steer(steer);
			StopLineDetect();
			if(StartDelay>=StartDelaySec)
				SpeedSet();
			speed_control();
			if(Up_Flag==1)
			{
				EMIOS_0.CH[9].CBDR.R = Openloop_Speed+20;
			}
		}
		Flag=0;
		Senddata();
	}
}
void OpenLoopMode()
{
	kp2-=0.2;
	kp3-=0.2;
	for (;;) 
		{
			Openloop_Speed=88;
			Key_Detect_Compensator();
			if(Flag==1)
			{
				sensor_display();
				steer=STEER_HELM_CENTER+LocPIDCal()+5;
				if(steer<=STEER_HELM_CENTER-240)
					steer=STEER_HELM_CENTER-250;
				if(steer>=STEER_HELM_CENTER+240)
					steer=STEER_HELM_CENTER+250;
				Dis_Num(64,3,(WORD)steer,4);
				if(StartDelay>=StartDelaySec)
					EMIOS_0.CH[9].CBDR.R = Openloop_Speed-10;
				if(Up_Flag==1)
				{
					steer=STEER_HELM_CENTER+13;
					EMIOS_0.CH[9].CBDR.R = Openloop_Speed-5;
				}
				if(StopFlag)
					EMIOS_0.CH[9].CBDR.R = 0;
				SET_steer(steer);
				StopLineDetect();
			//	if(StartDelay>=StartDelaySec)
					//SpeedSet();
				//speed_control();
			}
			Flag=0;
			Senddata();
		}
}
