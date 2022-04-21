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

#define RD_PIN    		GET_PIN(B, 6)
#define CAN_DEV_NAME       "can1"
#define UART1_DEV_NAME       "uart1"
#define UART2_DEV_NAME       "uart2"
#define UART5_DEV_NAME       "uart5"

static rt_device_t l_can_dev;
static struct rt_semaphore l_can_sem;

static rt_device_t l_uart1_dev;
static struct rt_semaphore l_uart1_sem;

static rt_device_t l_uart2_dev;
static struct rt_semaphore l_uart2_sem;

static rt_device_t l_uart5_dev;
static struct rt_semaphore l_uart5_sem;


static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&l_can_sem);

    return RT_EOK;
}



static rt_err_t uart1_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (size > 0)
    {
        rt_sem_release(&l_uart1_sem);
    }
    return RT_EOK;
}

static rt_err_t uart2_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (size > 0)
    {
        rt_sem_release(&l_uart2_sem);
    }
    return RT_EOK;
}

static rt_err_t uart5_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (size > 0)
    {
        rt_sem_release(&l_uart5_sem);
    }
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

static void uart1_entry(void *parameter)
{
	rt_err_t res;
	
	l_uart1_dev = rt_device_find(UART1_DEV_NAME);
	if (!l_uart1_dev)
	{
			LOG_E("find %s failed!\n", UART1_DEV_NAME);
			return;
	}
	rt_sem_init(&l_uart1_sem, "l_uart1_sem", 0, RT_IPC_FLAG_FIFO);
	
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	config.baud_rate = BAUD_RATE_115200;        
	config.data_bits = DATA_BITS_8;           
	config.stop_bits = STOP_BITS_1;           
	config.bufsz     = 128;                   
	config.parity    = PARITY_NONE;           


	rt_device_control(l_uart1_dev, RT_DEVICE_CTRL_CONFIG, &config);
	
	res = rt_device_open(l_uart1_dev, RT_DEVICE_FLAG_INT_RX);
	RT_ASSERT(res == RT_EOK);
	rt_device_set_rx_indicate(l_uart1_dev, uart1_rx_ind);
	
	while(1) {
		char buf[128] = "";
		rt_size_t  size;
		
		rt_sem_take(&l_uart1_sem, RT_WAITING_FOREVER);
		size = rt_device_read(l_uart1_dev, 0, buf, sizeof(buf));

		rt_kprintf(buf);
		
		rt_pin_write(RD_PIN, PIN_HIGH);
		rt_device_write(l_uart1_dev, 0, buf, size);
		rt_pin_write(RD_PIN, PIN_LOW);
	}
}

static void uart2_entry(void *parameter)
{
	rt_err_t res;
	
	l_uart2_dev = rt_device_find(UART2_DEV_NAME);
	if (!l_uart2_dev)
	{
			LOG_E("find %s failed!\n", UART2_DEV_NAME);
			return;
	}
	rt_sem_init(&l_uart2_sem, "l_uart2_sem", 0, RT_IPC_FLAG_FIFO);
	
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	config.baud_rate = BAUD_RATE_115200;        
	config.data_bits = DATA_BITS_8;           
	config.stop_bits = STOP_BITS_1;           
	config.bufsz     = 128;                   
	config.parity    = PARITY_NONE;           


	rt_device_control(l_uart2_dev, RT_DEVICE_CTRL_CONFIG, &config);
	
	res = rt_device_open(l_uart2_dev, RT_DEVICE_FLAG_INT_RX);
	RT_ASSERT(res == RT_EOK);
	rt_device_set_rx_indicate(l_uart2_dev, uart2_rx_ind);
	
	while(1) {
		char buf[128] = "";
		rt_size_t  size;
		
		rt_sem_take(&l_uart2_sem, RT_WAITING_FOREVER);
		size = rt_device_read(l_uart2_dev, 0, buf, sizeof(buf));

		rt_kprintf(buf);
		
		rt_device_write(l_uart2_dev, 0, buf, size);
	}
}

static void uart5_entry(void *parameter)
{
	rt_err_t res;
	
	l_uart5_dev = rt_device_find(UART5_DEV_NAME);
	if (!l_uart5_dev)
	{
			LOG_E("find %s failed!\n", UART5_DEV_NAME);
			return;
	}
	rt_sem_init(&l_uart5_sem, "l_uart5_sem", 0, RT_IPC_FLAG_FIFO);
	
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	config.baud_rate = BAUD_RATE_115200;        
	config.data_bits = DATA_BITS_8;           
	config.stop_bits = STOP_BITS_1;           
	config.bufsz     = 128;                   
	config.parity    = PARITY_NONE;           


	rt_device_control(l_uart5_dev, RT_DEVICE_CTRL_CONFIG, &config);
	
	res = rt_device_open(l_uart5_dev, RT_DEVICE_FLAG_INT_RX);
	RT_ASSERT(res == RT_EOK);
	rt_device_set_rx_indicate(l_uart5_dev, uart5_rx_ind);
	
	while(1) {
		char buf[128] = "";
		rt_size_t  size;
		
		rt_sem_take(&l_uart5_sem, RT_WAITING_FOREVER);
		size = rt_device_read(l_uart5_dev, 0, buf, sizeof(buf));

		rt_kprintf(buf);
		
		rt_device_write(l_uart5_dev, 0, buf, size);
	}
}

int serial_init(void)
{
	rt_thread_t tid_can = rt_thread_create("can", can_entry, RT_NULL,
                           1*1024, 20, 20);
	if (tid_can != RT_NULL)
	{
			rt_thread_startup(tid_can);
	}
	
	rt_thread_t tid_uart1 = rt_thread_create("uart1", uart1_entry, RT_NULL,
                           1*1024, 20, 20);
	if (tid_uart1 != RT_NULL)
	{
			rt_thread_startup(tid_uart1);
	}
	
	rt_thread_t tid_uart2 = rt_thread_create("uart2", uart2_entry, RT_NULL,
                           1*1024, 20, 20);
	if (tid_uart2 != RT_NULL)
	{
			rt_thread_startup(tid_uart2);
	}
	
	rt_thread_t tid_uart5 = rt_thread_create("uart5", uart5_entry, RT_NULL,
                           1*1024, 20, 20);
	if (tid_uart5 != RT_NULL)
	{
			rt_thread_startup(tid_uart5);
	}
	

	return RT_EOK;
}
