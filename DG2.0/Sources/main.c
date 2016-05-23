#include "includes.h"
int Flag=0,wait=9;
signed int steer=0;





void main(void)
{
	initALL();
	while(wait>0);
	Set_Middlepoint();
	for (;;) 
	{
		if(Flag==1)
		{
			sensor_display();
			position();
			steer=STEER_HELM_CENTER+LocPIDCal();
			//steer=STEER_HELM_CENTER+Steer();
			if(steer<694)
				steer=684;
			if(steer>=1033)
				steer=1055;
			Dis_Num(64,3,(WORD)steer,5);
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
	Get_speed();
	if(wait>0)
		wait--;
	PIT.CH[0].TFLG.B.TIF = 1;
}


