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

#define DBG_LEVEL                      DBG_INFO
#include <rtdbg.h>

#include "spi_flash_sfud.h"
#include "wiz.h"
#include "crc32.h"
#include "AP_ctrl.h"
#include "pid_ctrl.h"


//#define AP_DEBUG

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

typedef enum
{
  PWM_Vpwm_1 = 0U,
	PWM_Vpwm_2,
	PWM_Vpwm_3,
	PWM_Vpwm_4,
	PWM_Mpwm_1,
	PWM_Mpwm_2,
	PWM_Ipwm_1,
	PWM_Ipwm_2
}PWM_CHN;


typedef struct tag_pwm_handSet{
	TIM_HandleTypeDef *hTim;
	uint32_t Channel;
}pwm_handSet;

static const pwm_handSet pwmhs[] = {
	{&htim3, TIM_CHANNEL_1},
	{&htim3, TIM_CHANNEL_2},
	{&htim3, TIM_CHANNEL_3},
	{&htim3, TIM_CHANNEL_4},
	{&htim2, TIM_CHANNEL_3},
	{&htim2, TIM_CHANNEL_4},
	{&htim2, TIM_CHANNEL_2},
	{&htim2, TIM_CHANNEL_1},
};

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

void pwm_set(TIM_HandleTypeDef *hTim, uint32_t Channel, float duty, int start)
{
	TIM_OC_InitTypeDef sConfig;
	
	sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;


  /* Set the pulse value for channel 1 */
	if(duty >= 1)
	{
		sConfig.Pulse = hTim->Init.Period+1;
	}
	else if(duty < 0)
	{
		sConfig.Pulse = 0;
	}
	else
	{
		sConfig.Pulse = (hTim->Init.Period)*duty;
	}
	
	if(HAL_TIM_ReadCapturedValue(hTim, Channel) == sConfig.Pulse && start != 1)
	{
		return;
	}
	
	__HAL_TIM_SetCompare(hTim, Channel, sConfig.Pulse);
	
	if(start == 1)
		if (HAL_TIM_PWM_Start(hTim, Channel) != HAL_OK)
		{
			/* PWM Generation Error */
			Error_Handler();
		}
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

void pwm_sample_set(uint32_t Channel, float duty, int start)
{
	pwm_set(pwmhs[Channel].hTim, pwmhs[Channel].Channel, duty, start);
}


uint16_t ad_buf[80] = {0};

void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

double target_v = 2.5;

static void Vout_entry(void *parameter)
{
	pwm_sample_set(PWM_Vpwm_1, 0.0, 1);
	pwm_sample_set(PWM_Vpwm_2, 0.0, 1);
	pwm_sample_set(PWM_Vpwm_3, 0.0, 1);
	pwm_sample_set(PWM_Vpwm_4, 0.0, 1);
	
	
	pwm_sample_set(PWM_Ipwm_1, 0.0, 1);
	pwm_sample_set(PWM_Ipwm_2, 0.0, 1);
	
	pwm_sample_set(PWM_Mpwm_1, 0.0, 1);
	pwm_sample_set(PWM_Mpwm_2, 0.0, 1);
	
	HAL_ADCEx_Calibration_Start(&hadc1); 
	HAL_ADCEx_Calibration_Start(&hadc2); 
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)ad_buf, sizeof(ad_buf)/sizeof(uint32_t));
	
	
	handle_PC pid_parm[4] = {0};
	for(int i=0; i<sizeof(pid_parm)/sizeof(pid_parm[0]); i++){
		pid_parm[i].m_pp.kp = 0.3;
		pid_parm[i].m_pp.ki = 0.3;
		pid_parm[i].m_pp.kd = 0.01;
		pid_parm[i].m_pp.ii_up_max = 10;
		pid_parm[i].m_pp.ii_dn_min = 0;
		pid_parm[i].m_pp.integral_inhibition = 1;
		pid_parm[i].m_pp.output_max = 10;
		pid_parm[i].m_pp.output_min = 0;
		rt_snprintf(pid_parm[i].name, sizeof(pid_parm[i].name), "Vout_%d", i+1);
	}
	
	while(1)
	{
		double Vout_actual[4] = {0};
		int array_size = sizeof(ad_buf)/sizeof(ad_buf[0])/8;
		
		for(int i=0; i<array_size; i++){
			for(int j=0; j<4; j++){
				Vout_actual[j] += ad_buf[i*8+j];
			}
		}
		
		for(int i=0; i<4; i++){
			Vout_actual[i] = Vout_actual[i] / array_size /4096.0*3.3 / (1/3.4);
			double perc = PC_realize(&pid_parm[i], Vout_actual[i], target_v) / 10.0;
			
#ifdef AP_DEBUG			
			char buf[128] = "";
			snprintf(buf, sizeof(buf), "perc_%d: %.2f", i, perc);
			LOG_I(buf);
#endif			
			pwm_sample_set(PWM_Vpwm_1+i, perc, 0);
		}
		
		rt_thread_mdelay(60);
	}
	

//analog test
//	float x = 0;
//	while(0){
//		x += 0.001;
//		for(int i=0; i<4; i++){
//			pwm_sample_set(PWM_Vpwm_1+i, x, 0);
//		}
//		
//		for(int i=0; i<2; i++){
//			pwm_sample_set(PWM_Ipwm_1+i, x, 0);
//		}
//			
//		rt_thread_mdelay(20);
//		
//		for(int i=0; i<4; i++){
//			char buf[128] = "";
//			snprintf(buf, sizeof(buf), "Vout_%d:%.2f  Ain_%d:%.2f duty:%.2f", i+1, ad_buf[0]/4096.0*3.3 / (1/3.4), i+1, ad_buf[4+i]/4096.0*3.3 / (40.2/140.2), x);
//			LOG_I(buf);
//		}
//		
//		for(int i=0; i<2; i++){
//			char buf[128] = "";
//			snprintf(buf, sizeof(buf), "Iout_%d:%.2f  Ain_%d:%.2f duty:%.2f", i+1, x*20.0, i+3, ad_buf[6+i]/4096.0*3.3 / (40.2/140.2) / 250 * 1000, x);
//			LOG_I(buf);
//		}
//		
//		if(x >= 1)
//			x = 0;
//	}
}

int AP_device_init()
{
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	
	return 0;
}

int AP_init(void)
{
	rt_thread_t tid = rt_thread_create("Vout", Vout_entry, RT_NULL,
                           4*1024, 8, 20);
	
	if (tid != RT_NULL)
	{
			rt_thread_startup(tid);
	}

	return RT_EOK;
}

INIT_BOARD_EXPORT(AP_device_init);

