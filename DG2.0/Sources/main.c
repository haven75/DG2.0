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
			if(steer<=920)
				steer=890;
			if(steer>=1290)
				steer=1310;
			Dis_Num(64,3,(WORD)steer,5);
			if(Up_Flag==1)
				steer=STEER_HELM_CENTER;
			SET_steer(steer);
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
	
	PIT.CH[0].TFLG.B.TIF = 1;
}


