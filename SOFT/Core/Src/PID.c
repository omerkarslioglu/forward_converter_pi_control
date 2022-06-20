/*
* Designed by Omer Karslioglu
* PID algorithm (just used PI)
*/

#include "PID.h"

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->pid_p	= 0.0f;
	pid->pid_i	= 0.0f;

	pid->error	= 0.0f;

	pid->PID = 0.0f;

	pid->previousError = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, int measurement, int pwm_resolution, int adc_resolution) {

	/*
	* Error signal
	*/
    pid->error = -setpoint+measurement;


	/*
	* Proportional
	*/
    pid->pid_p = pid->kp*pid->error;


	//integrator:
	if(-10<pid->error && pid->error<10)
	{
		pid->pid_i=pid->pid_i+(pid->ki*pid->error);
		// anti wind-up
		if(pid->pid_i>pid->max_lim_integral)
			pid->pid_i=pid->max_lim_integral;
		else if(pid->pid_i<-pid->max_lim_integral)
			pid->pid_i=-pid->max_lim_integral;
	}

	// Derivator
	//pid_d=kd*((error-previousError)/dt);


	/*
	* Compute output and apply limits
	*/
	//PID out:
	pid->PID=((pid->pid_p+pid->pid_i)*pwm_resolution)/adc_resolution;

	//PID limit:
	if(pid->PID<-pid->max_lim_PID)
	{
	  pid->PID=-pid->max_lim_PID;
	}
	if(pid->PID>pid->max_lim_PID)
	{
	  pid->PID=pid->max_lim_PID;
	}

	/* Store error and measurement for later use */
    pid->previousError = pid->error;

	/* Return controller output */
    return pid->PID;
}
