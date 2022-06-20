#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

	/* Controller gains */
	float kp;
	float ki;

	float max_lim_integral;

	float max_lim_PID;

	float pid_p; // proportional output
	float pid_i; // integrator output

	float error;

	float PID; // pid output

	float previousError;
} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, int measurement, int pwm_resolution, int adc_resolution);

#endif
