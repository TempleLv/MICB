#ifndef __PID_CTRL__
#define __PID_CTRL__

#include <stdint.h>

typedef struct tag_pc_param {
	double kp;
	double ki;
	double kd;
	double ii_up_max;
	double ii_dn_min;
	int integral_inhibition;
	double output_max;
	double output_min;
} pc_param;

typedef struct tag_handle_PC {
	pc_param m_pp;
	char name[64];
	double m_err;
	double m_last_err;
	double m_integral;
} handle_PC;


int pid_init(handle_PC *handle, pc_param *param);

double PC_realize(handle_PC *h, double actual_value, double target_value);


#endif

