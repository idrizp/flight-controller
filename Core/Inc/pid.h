#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct pid {
	float prev_error;
	float integrator;

	float ki;
	float kp;
	float kd;

	float dt;
} pid;

void update_pid(pid* pid, float error, float* control);

#endif /* INC_PID_H_ */
