#include "includes.h"
unsigned int Flag=0,wait=9,INTC_Time=0,Ramp_Steer=0;
signed int steer=0,delay_count=0,StartDelay=0;
#define StartDelaySec 100


void FastSpeedMode();
void MiddleSpeedMode();
void SlowSpeedMode();
void OpenLoopMode();

void main(void)
 {
	initALL();
	while(wait>0);
	if(switch4==1)
		pause=1;
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
	INTC_Time++;
	if(INTC_Time==5)
	{
		Flag=1;
		frequency_measure();
		//Get_speed();
		if(wait>0)
			wait--;
		if(1)
		{
			if(delay_count<500)
				delay_count++;    //·ÀÖ¹¿ª»ú´¥·¢
			else Ramp_Detect();
			if(Ramp_Flag==1)
				Ramp_Time++;
			if(Ramp_Time>65)
			{
				Up_Flag=2;
				//Ramp_Flag=0;
			}
			if(Ramp_Time>60&&Ramp_Time<100)
			{
				if(steer<=STEER_HELM_CENTER-30)
					steer=STEER_HELM_CENTER-30;
				if(steer>=STEER_HELM_CENTER+30)
					steer=STEER_HELM_CENTER+30;
				Ramp_Steer=1;
			}
			if(Ramp_Time>100)
				Ramp_Steer=0;
			if(Ramp_Time>200)
			{
				Up_Flag=0;
				Down_Flag=0;
				Ramp_Flag=0;
				Uphill=0;
				Ramp_Time=0;
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
		INTC_Time=0;
	}
	Get_speed();
	if(switch2!=0||switch1!=0)
	{
		if(StartDelay>=StartDelaySec||pause==1)
			SpeedSet();
		speed_control();
	}
	PIT.CH[0].TFLG.B.TIF = 1;
}

/*void Pit1ISR()
{
	Get_speed();
	if(StartDelay>=StartDelaySec)
		SpeedSet();
	speed_control();
}*/

void FastSpeedMode()
{	
	kp1=7;		kd1=40;  
	kp2=4.4;	kd2=45;
	kp3=3.2;	kd3=50;
	kp4=2.4;	kd4=55;
	speed_kp=5.4;
	speed_ki=1.2;
	speed_kd=3;
	speed1=88;
	speed2=68;
	speed3=56;
	speed4=54;
	speed5=46;
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-230)
				steer=STEER_HELM_CENTER-240;
			if(steer>=STEER_HELM_CENTER+230)
				steer=STEER_HELM_CENTER+240;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER+6;
			if(!Ramp_Steer)
				SET_steer(steer);
			StopLineDetect();
		/*	if(StartDelay>=StartDelaySec)
				SpeedSet();
			speed_control();*/
		}
		Flag=0;
		Senddata();
	}
}
                   
void MiddleSpeedMode()
{
	kp1=6.9;	kd1=40;  
	kp2=4.4;	kd2=45;
	kp3=3.2;	kd3=50;
	kp4=2.4;	kd4=55;
	speed_kp=5.4;
	speed_ki=1.2;
	speed_kd=3;
	speed1=80;
	speed2=64;
	speed3=56;
	speed4=50;
	speed5=42;
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-230)
				steer=STEER_HELM_CENTER-240;
			if(steer>=STEER_HELM_CENTER+230)
				steer=STEER_HELM_CENTER+240;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER+6;
			if(!Ramp_Steer)
				SET_steer(steer);
			StopLineDetect();
		/*	if(StartDelay>=StartDelaySec)
				SpeedSet();
			speed_control();*/
		}
		Flag=0;
		Senddata();
	}
}

void SlowSpeedMode()
{
	kp1=6.9;		kd1=32;  
	kp2=4.4;	kd2=34;
	kp3=3.3;	kd3=38;
	kp4=2.4;	kd4=42;
	
	speed_kp=5.4;
	speed_ki=2;
	speed_kd=4;
	speed1=70;
	speed2=56;
	speed3=50;
	speed4=46;
	speed5=42;
	Open=1;
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=STEER_HELM_CENTER-230)
				steer=STEER_HELM_CENTER-240;
			if(steer>=STEER_HELM_CENTER+230)
				steer=STEER_HELM_CENTER+240;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
			{
				steer=STEER_HELM_CENTER+6;
			}
			if(!Ramp_Steer)
				SET_steer(steer);			
			StopLineDetect();
		/*	if(StartDelay>=StartDelaySec)
				SpeedSet();
			speed_control();*/
		}
		Flag=0;
		Senddata();
	}
}
void OpenLoopMode()
{
	kp2-=0.2;
	kp3-=0.2;
	kd4+=10;
	for (;;) 
		{
			Openloop_Speed=92;
			Key_Detect_Compensator();
			if(Flag==1)
			{
				sensor_display();
				steer=STEER_HELM_CENTER+LocPIDCal()+8;
				if(steer<=STEER_HELM_CENTER-230)
					steer=STEER_HELM_CENTER-240;
				if(steer>=STEER_HELM_CENTER+230)
					steer=STEER_HELM_CENTER+240;
				Dis_Num(64,3,(WORD)steer,4);
				if(StartDelay>=StartDelaySec||switch5==0)
					EMIOS_0.CH[10].CBDR.R = 40;//Openloop_Speed;
				if(Up_Flag==1)
				{
					steer=STEER_HELM_CENTER+7;
					EMIOS_0.CH[9].CBDR.R = Openloop_Speed-25;
				}
				if(StopFlag)
					EMIOS_0.CH[9].CBDR.R = 0;
				if(!Ramp_Steer)
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
