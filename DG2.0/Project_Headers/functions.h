/*
 * functions.h
 *
 *  Created on: Feb 27, 2016
 *      Author: Administrator
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

void frequency_measure();
void InitspPID();
void InitsePID();
signed int LocPIDCal();
void sensor_display();
void Get_speed();

void SAIC1_inter(void);
//void SAIC2_inter(void);
void Set_Middlepoint();
void SendHex(unsigned char hex);
void Senddata();
void SpeedSet();
void speed_control();
void Key_Detect_Compensator();
signed int Steer();
void delay();
unsigned int ADC_GetValueChannel();
unsigned int Get_Angle();
void StopLineDetect();


void Ramp_Detect();



extern unsigned char Left_Compensator, Right_Compensator,Open;
extern float Middle_Compensator;
extern unsigned int Ramp_Time_Delay,pause;
extern unsigned int count,min_count;
extern unsigned int Uphill,Downhill,Up_Flag,Down_Flag,Ramp_Flag,Ramp_Time,
					speed1,speed2,speed3,speed4,speed5;
extern unsigned char StartFlag,StopFlag,RunFlag,Stop;
extern float kp1,kp2,kp3,kp4,kd1,kd2,kd3,kd4,speed_kp,speed_ki,speed_kd;
#endif /* FUNCTIONS_H_ */
