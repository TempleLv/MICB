/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "pid_ctrl.h"
#include <stdio.h>

#define DBG_LEVEL                      DBG_INFO
#include <rtdbg.h>

//#define PID_DEBUG


int pid_init(handle_PC *h, pc_param *param) 
{
	h->m_pp = *param;
	
	return 0;
}

double PC_realize(handle_PC *h, double actual_value, double target_value) {
	
	double ret = 0;

	if (h != NULL) {
		pc_param pp = h->m_pp;

		h->m_err = target_value - actual_value;

		if (h->m_pp.integral_inhibition != 0) {
			ret = pp.kp * h->m_err + pp.ki * (h->m_integral + h->m_err) + pp.kd * (h->m_err - h->m_last_err);
//            if ((ret <= h->m_pp.ii_up_max && ret >= h->m_pp.ii_dn_min) ||
//                (ret > h->m_pp.ii_up_max && h->m_err < 0) ||
//                (ret < h->m_pp.ii_dn_min && h->m_err > 0)) {
//                h->m_integral += h->m_err;
//            }
			if (ret > h->m_pp.ii_up_max) {
				h->m_integral = (pp.ki * (h->m_integral + h->m_err) - (ret - h->m_pp.ii_up_max)) / pp.ki;
				if(h->m_integral < 0)
					h->m_integral = 0;
			} else if (ret < h->m_pp.ii_dn_min) {
				h->m_integral = (pp.ki * (h->m_integral + h->m_err) - (ret - h->m_pp.ii_dn_min)) / pp.ki;
				if(h->m_integral > 0)
					h->m_integral = 0;
			} else
				h->m_integral += h->m_err;

				ret = pp.kp * h->m_err + pp.ki * h->m_integral + pp.kd * (h->m_err - h->m_last_err);

		} else {
			h->m_integral += h->m_err;
			ret = pp.kp * h->m_err + pp.ki * h->m_integral + pp.kd * (h->m_err - h->m_last_err);
		}
		
#ifdef PID_DEBUG		
		char buf[128] = "";
		snprintf(buf, sizeof(buf), "name:%s p:%.2f i:%.2f d:%.2f last_err:%.2f err:%.2f", h->name, 
			pp.kp * h->m_err, pp.ki * h->m_integral, pp.kd * (h->m_err - h->m_last_err), h->m_last_err, h->m_err);
		LOG_I(buf);
#endif
		
		if(ret > h->m_pp.output_max)
			ret = h->m_pp.output_max;
		else if(ret < h->m_pp.output_min)
			ret = h->m_pp.output_min;

		h->m_last_err = h->m_err;
	}

	return ret;
}

