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
#include "drv_spi.h"
#include "spi_flash_sfud.h"
#include "wiz.h"
#include "crc32.h"

/* defined the LED0 pin: PB1 */
//#define LED0_PIN    GET_PIN(B, 1)
#define RLY_1_PIN    	GET_PIN(C, 1)
#define RLY_2_PIN    	GET_PIN(C, 2)
#define RLY_3_PIN    	GET_PIN(C, 3)
#define RD_PIN    		GET_PIN(B, 6)

/* set WIZnet device MAC address */
void wiz_user_config_mac(char *mac_buf, rt_uint8_t buf_len)
{
	RT_ASSERT(mac_buf != RT_NULL);
	RT_ASSERT(buf_len > 0);
	uint32_t uid[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};

	rt_memset(mac_buf, 0x0, buf_len);
	uint32_t mac = crc32_cal((u8 *)uid, sizeof(uid));
	uint8_t *pMac = (uint8_t *)&mac;
	rt_snprintf(mac_buf, buf_len, "00-08-%02X-%02X-%02X-%02X", pMac[0], pMac[1], pMac[2], pMac[3]);
}

int MICB_device_init()
{
	//init RLY
	rt_pin_mode(RLY_1_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(RLY_1_PIN, PIN_LOW);
	rt_pin_mode(RLY_2_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(RLY_2_PIN, PIN_LOW);
	rt_pin_mode(RLY_3_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(RLY_3_PIN, PIN_LOW);
	
	//init 485
	rt_pin_mode(RD_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(RD_PIN, PIN_LOW);
	
	//init GD25Q16 spi
	rt_hw_spi_device_attach("spi1", "spi10", GPIOA, GPIO_PIN_15);
	
	//init W5500 spi
	rt_hw_spi_device_attach("spi2", "spi20", GPIOB, GPIO_PIN_12);
	
	return 0;
}


int main(void)
{
	
	rt_sfud_flash_probe("GD25Q16", "spi10");


	while (1)
	{
		rt_thread_mdelay(500);
	}

	return RT_EOK;
}


INIT_DEVICE_EXPORT(MICB_device_init);
