#include "pid.h"

void update_pid(pid* pid, float error, float* control) {
	if ((error > 0 && pid->prev_error < 0) || (error < 0 && pid->prev_error > 0)) {
		// we just crossed zero
		pid->integrator = 0;
		// this prevents integrator windup
	}
	pid->integrator = pid->integrator + error;
	// PID(proportional, integral, derivative)

	// TODO: Derivative might need to be LPF'd so that we don't have really high-frequency changes that can cause a lot of issues for us!
	*control = pid->kp * error + pid->ki * pid->integrator * pid->dt + ((error - pid->prev_error) * pid->kd / pid->dt);
	pid->prev_error = error;
}
