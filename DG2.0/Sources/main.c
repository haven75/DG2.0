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
<<<<<<< HEAD
			//steer=STEER_HELM_CENTER+Steer();
			if(steer<684)
				steer=676;
			if(steer>=1023)
				steer=1045;
=======
			if(steer<682)
				steer=672;
			if(steer>=1021)
				steer=1043;
>>>>>>> 62a5ad70a917be7220e35a2c1e7f6fe5767c2abf
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


