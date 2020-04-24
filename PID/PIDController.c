#include "PIDcontroller.h"

float map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculatePID(float roll, float pitch, float yaw,long elapsedTime,float * pwm_L_F,float * pwm_L_B, pwm_R_F,float * pwm_R_B);
{

	/*First calculate the error between the desired angle and 
	*the real measured angle*/
	roll_error = roll - roll_desired_angle;
	pitch_error = pitch - pitch_desired_angle; 
	yaw_error = yaw - yaw_desired_angle; 
	/*Next the proportional value of the PID is just a proportional constant
	*multiplied by the error*/
	roll_pid_p = roll_kp*roll_error;
	pitch_pid_p = pitch_kp*pitch_error;
	yaw_pid_p = yaw_kp*yaw_error;
	/*The integral part should only act if we are close to the
	desired position but we want to fine tune the error. That's
	why I've made a if operation for an error between -2 and 2 degree.
	To integrate we just sum the previous integral value with the
	error multiplied by  the integral constant. This will integrate (increase)
	the value each loop till we reach the 0 point*/
	if(-3 < roll_error <3)
	{
  		roll_pid_i = roll_pid_i+(roll_ki*roll_error);  
	}
	if(-3 < pitch_error <3)
	{
  		pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);  
	}
	if(-3 < yaw_error <3)
	{
  		yaw_pid_i = yaw_pid_i+(yaw_ki*yaw_error);  
	}
	
	/*The last part is the derivate. The derivate acts upon the speed of the error.
	As we know the speed is the amount of error that produced in a certain amount of
	time divided by that time. For taht we will use a variable called previous_error.
	We substract that value from the actual error and divide all by the elapsed time. 
	Finnaly we multiply the result by the derivate constant*/
	roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
	pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);
	yaw_pid_d = yaw_kd*((yaw_error - yaw_previous_error)/elapsedTime);
	/*The final PID values is the sum of each of this 3 parts*/
	roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
	pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
	yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;
	
	if(roll_PID < -1000){roll_PID=-1000;}
	if(roll_PID > 1000) {roll_PID=1000; }
	if(pitch_PID < -1000){pitch_PID=-1000;}
	if(pitch_PID > 1000) {pitch_PID=1000;}
	if(yaw_PID < -1000){yaw_PID=-1000;}
	if(yaw_PID > 1000) {yaw_PID=1000;}
	
	/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
	*pwm_R_F  = 115 - roll_PID - pitch_PID;   // Yaw PID is not added yet !
	*pwm_R_B  = 115 - roll_PID + pitch_PID;
	*pwm_L_B  = 115 + roll_PID + pitch_PID;
	*pwm_L_F  = 115 + roll_PID - pitch_PID;
	
	// Map the values according to timer's frequency, ARR and prescaler
	pwm_R_F = map(pwm_R_F,-1000,1000,500,1000);
	pwm_R_B = map(pwm_R_B,-1000,1000,500,1000);
	pwm_L_F = map(pwm_L_F,-1000,1000,500,1000);
	pwm_L_B = map(pwm_L_B,-1000,1000,500,1000);


	/*Once again we map the PWM values to be sure that we won't pass the min
	and max values. Yes, we've already maped the PID values. But for example, for 
	throttle value of 1300, if we sum the max PID value we would have 2300us and
	that will mess up the ESC.*/
	//Right front
	if(pwm_R_F < 500)
	{
  		pwm_R_F= 500;
	}
	if(pwm_R_F > 1000)	
	{
  		pwm_R_F=1000;
	}

	//Left front
	if(pwm_L_F < 500)
	{
  		pwm_L_F= 500;
	}
	if(pwm_L_F > 1000)
	{
  		pwm_L_F=1000;
	}

	//Right back
	if(pwm_R_B < 500)
	{
  	pwm_R_B= 500;
	}
	if(pwm_R_B > 1000)
	{
  	pwm_R_B=1000;
	}

	//Left back
	if(pwm_L_B < 500)
	{
  	pwm_L_B= 500;
	}
	if(pwm_L_B > 1000)
	{
  	pwm_L_B=1000;
	}

	roll_previous_error = roll_error; //Remember to store the previous error.
	pitch_previous_error = pitch_error; //Remember to store the previous error.
	
}
