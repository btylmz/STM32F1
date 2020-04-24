#include "stdio.h"
#include "stdbool.h"
#include "math.h"


#define roll_desired_angle 0
#define pitch_desired_angle 0
#define yaw_desired_angle 0


//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=0.7;//3.55
double roll_ki=0.006;//0.003
double roll_kd=1.2;//2.05

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=0.72;//3.55
double pitch_ki=0.006;//0.003
double pitch_kd=1.22;//2.05

//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p=0;
float yaw_pid_i=0;
float yaw_pid_d=0;
///////////////////////////////YAW PID CONSTANTS///////////////////
double yaw_kp=0.72;//3.55
double yaw_ki=0.006;//0.003
double yaw_kd=1.22;//2.05

float map(long x, long in_min, long in_max, long out_min, long out_max);
void calculatePID(float roll, float pitch, float yaw,float * pwm_L_F,float * pwm_L_B, pwm_R_F,float * pwm_R_B);



