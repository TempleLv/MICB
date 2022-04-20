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
#include <rtdbg.h>
#include "AP_ctrl.h"

#define CAN_DEV_NAME       "can1"

static rt_device_t l_can_dev;
static struct rt_semaphore l_can_sem;


static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&l_can_sem);

    return RT_EOK;
}


static void can_entry(void *parameter)
{
	rt_err_t res;
	
	l_can_dev = rt_device_find(CAN_DEV_NAME);
	if (!l_can_dev)
	{
			LOG_E("find %s failed!\n", CAN_DEV_NAME);
			return;
	}
	rt_sem_init(&l_can_sem, "l_can_sem", 0, RT_IPC_FLAG_FIFO);
	
	res = rt_device_open(l_can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
	RT_ASSERT(res == RT_EOK);
	res = rt_device_control(l_can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN250kBaud);
	RT_ASSERT(res == RT_EOK);
	rt_device_set_rx_indicate(l_can_dev, can_rx_call);
	
	while(1) {
		struct rt_can_msg rxmsg = {0};
		rt_size_t  size;
		
		rt_sem_take(&l_can_sem, RT_WAITING_FOREVER);
		rt_device_read(l_can_dev, 0, &rxmsg, sizeof(rxmsg));
		
		rt_kprintf("ID:%04x ", rxmsg.id);
		for (int i = 0; i < 8; i++)
		{
				rt_kprintf("%02x ", rxmsg.data[i]);
		}
		rt_kprintf("\n");
		
		size = rt_device_write(l_can_dev, 0, &rxmsg, sizeof(rxmsg));
    if (size == 0)
    {
        LOG_E("can dev write data failed!\n");
    }
	}
}

int serial_init(void)
{
	rt_thread_t tid = rt_thread_create("can", can_entry, RT_NULL,
                           2*1024, 20, 20);
	
	if (tid != RT_NULL)
	{
			rt_thread_startup(tid);
	}

	return RT_EOK;
}
