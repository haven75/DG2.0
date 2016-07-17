#include "includes.h"
int Flag=0,wait=9;
signed int steer=0,delay_count=0;




void main(void)
 {
	initALL();
	while(wait>0);
	Set_Middlepoint();
	for (;;) 
	{
		Key_Detect_Compensator();
		if(Flag==1)
		{
			sensor_display();
			steer=STEER_HELM_CENTER+LocPIDCal();
			if(steer<=901)
				steer=878;
			if(steer>=1311)
				steer=1320;
			Dis_Num(64,3,(WORD)steer,4);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER;
			SET_steer(steer);
			StopLineDetect();
			SpeedSet();
			speed_control();
		}
		Flag=0;
		Senddata();
	}
}


void Pit0ISR()     
{
	Flag=1;
	frequency_measure();
	Get_Angle();
	Get_speed();
	if(wait>0)
		wait--;
	
	if(delay_count<500)
		delay_count++;
	else Ramp_Detect();
	
	if(Ramp_Flag==1)
		Ramp_Time++;
	if(Ramp_Time>140)
	{
		Up_Flag=2;
		Ramp_Flag=0;
	}
		
	PIT.CH[0].TFLG.B.TIF = 1;
}


