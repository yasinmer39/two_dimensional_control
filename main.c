/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "stdbool.h"
//#include "A.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68<<1
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;


/* USER CODE BEGIN PV */
uint16_t USBGiden[100];
uint16_t pwm;
uint16_t pwm2;
float set = 0;
float kpacc = 12, ki = 0.1, kd = 0.2, kpgyro = 0.6;
double throttle = 1300;
float pid_p = 0, pid_i = 0, pid_d = 0, PID, PID2, pid_p1 = 0;
float error, previous_error, error_cascade;
unsigned long t_now, t_last;
float dt;
float gyro_cal[3];
int16_t acc_total_vector;
float angle_pitch_gyro, angle_roll_gyro;
float angle_pitch_acc, angle_roll_acc;
float angle_pitch_gyro_velocity;
float angle_pitch, angle_roll;
int16_t raw_temp;
float temp;
int i;
float prevtime, prevtime1, time1, elapsedtime1, prevtime2, time2, elapsedtime2;
HAL_StatusTypeDef set_gyro;
uint8_t data;
uint8_t buffer[2], tuffer[6], cuffer[6];
int16_t gyro_raw[3], acc_raw[3];
#define Elapsed_time 0.001
uint32_t system_clock_counter_1ms;
typedef union
{
	struct
	{
		uint8_t Hz_1000 : 1 ;
		uint8_t Hz_200 : 1;
		uint8_t Hz_100 : 1;
		uint8_t Hz_50 : 1;
		uint8_t Hz_20 : 1;
		uint8_t Hz_10 : 1;
		uint8_t Hz_5 : 1;
		uint8_t Hz_1 : 1;
	}Task;

	uint8_t all_u8;

}time_task_union;

 time_task_union Time;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback(void)
{
	system_clock_counter_1ms++;

	if( system_clock_counter_1ms % 1000 == 0)
	{
		Time.Task.Hz_1 = 1;
	}
	if( system_clock_counter_1ms % 200 == 0)
	{
		Time.Task.Hz_5 = 1;
	}
	if( system_clock_counter_1ms % 100 == 0)
	{
		Time.Task.Hz_10 = 1;
	}
	if( system_clock_counter_1ms % 50 == 0)
	{
		Time.Task.Hz_20 = 1;
	}
	if( system_clock_counter_1ms % 20 == 0)
	{
		Time.Task.Hz_50 = 1;
	}
	if( system_clock_counter_1ms % 10 == 0)
	{
		Time.Task.Hz_100 = 1;
	}
	if( system_clock_counter_1ms % 5 == 0)
	{
		Time.Task.Hz_200 = 1;
	}
	if( system_clock_counter_1ms % 1 == 0)
	{
		Time.Task.Hz_1000 = 1;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);


    HAL_Delay(2000);

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
    data = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);

    for(i=0; i<2000; i++)
      {
    	  prevtime2 = time2;
    	  time2 = HAL_GetTick();
    	  elapsedtime2 = (time2-prevtime)*1000.0;

    	  cuffer[0]=0x43;
    	  HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cuffer, 1, HAL_MAX_DELAY);
    	  HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, cuffer, 6, HAL_MAX_DELAY);

    	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
    	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
    	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

    	  gyro_cal[0] += gyro_raw[0];
    	  gyro_cal[1] += gyro_raw[1];
    	  gyro_cal[2] += gyro_raw[2];

    	  HAL_Delay(3);

      }

      gyro_cal[0] /= 2000.0;
      gyro_cal[1] /= 2000.0;
      gyro_cal[2] /= 2000.0;

      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
      HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  prevtime1 = time1;
	  	  	  time1 = HAL_GetTick();
	  	  	  elapsedtime1 = (time1-prevtime1)*1000.0;

	  	  	  tuffer[0] = 0x3B;
	  	  	  HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, tuffer, 1, HAL_MAX_DELAY);
	  	  	  HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, tuffer, 6, HAL_MAX_DELAY);

	  	  	  acc_raw[0] = (tuffer[0] << 8 | tuffer[1]);
	  	  	  acc_raw[1] = (tuffer[2] << 8 | tuffer[3]);
	  	  	  acc_raw[2] = (tuffer[4] << 8 | tuffer[5]);

	  	  	  buffer[0] = 0x41;
	  	  	  HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buffer, 1, HAL_MAX_DELAY);
	  	  	  HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buffer, 2, HAL_MAX_DELAY);

	  	  	  raw_temp = (buffer[0] << 8 | buffer[1]);
	  	  	  temp = (raw_temp / 340.0) + 36.53;

	  	  	  cuffer[0] = 0x43;
	  	  	  HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cuffer, 1, HAL_MAX_DELAY);
	  	  	  HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, cuffer, 6, HAL_MAX_DELAY);

	  	  	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
	  	  	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
	  	  	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

	  	  	  gyro_raw[0] = gyro_raw[0] - gyro_cal[0];
	  	  	  gyro_raw[1] -= gyro_cal[1];
	  	  	  gyro_raw[2] -= gyro_cal[2];

	  	  	  angle_pitch_gyro += gyro_raw[0] * 0.0000611;
	  	  	  angle_roll_gyro += gyro_raw[1] * 0.0000611;
	  	  	  angle_pitch_gyro_velocity = gyro_raw[0] * 0.0000611;

	  	  	  angle_pitch_gyro += angle_roll_gyro * sin(gyro_raw[2] * 0.000001066);
	  	  	  angle_roll_gyro -= angle_pitch_gyro * sin(gyro_raw[2] * 0.000001066);

	  	  	  acc_total_vector = sqrt((acc_raw[0]*acc_raw[0])+(acc_raw[1]*acc_raw[1])+(acc_raw[2]*acc_raw[2]));

	  	  	  angle_pitch_acc = asin((float)acc_raw[1]/acc_total_vector) * 57.296;
	  	  	  angle_roll_acc = asin((float)acc_raw[0]/acc_total_vector) * -57.296;

	  	  	  angle_pitch_acc -= 0.00;
	  	  	  angle_roll_acc -= 0.00;

	  	  	  if(set_gyro)
	  	  	  {
	  	  		  angle_pitch = angle_pitch_gyro * 0.9996 + angle_pitch_acc * 0.0004;
	  	  		  angle_roll = angle_roll_gyro * 0.9996 + angle_roll_acc * 0.0004;
	  	  	  }
	  	  	  else
	  	  	  {
	  	  		  angle_pitch = angle_pitch_acc;
	  	  		  set_gyro = true;
	  	  	  }

	  	  	  while((HAL_GetTick() - prevtime) * 1000.0 < 4000.0);
	  	  	  prevtime = HAL_GetTick();

	  		t_last = t_now;
	  		  t_now = HAL_GetTick();
	  		  dt = (t_now-t_last)/1000.0;

	  		  //disaridaki cascade
	  		if(Time.Task.Hz_200 == 1){
	  		  error = angle_pitch-set;
	  		  pid_p = kpacc*error;
	  		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	  		Time.Task.Hz_200 = 0;

	  		  //icerideki cascade
	  		}
	  		if(Time.Task.Hz_1000 == 1){
	  		  error_cascade = pid_p-angle_pitch_gyro_velocity;
	  		  pid_p1 = kpgyro*error_cascade;
	  		 if(-10 < error_cascade && error_cascade < 10)
	  				  {
	  					  pid_i = pid_i + (ki*error_cascade)*dt;
	  				  }
	  			  pid_d = kd*((error_cascade-previous_error)/dt);
	  			  PID =  pid_p1 + pid_i + pid_d;
	  			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  			Time.Task.Hz_1000 = 0;

	  		}



	  		  if(PID < -300)
	  		  {
	  			  PID = -300;
	  		  }
	  		  if(PID > 700)
	  		  {
	  			  PID = 700;
	  		  }
	  		  pwm = throttle + PID;
	  		  pwm2 = throttle - PID;
	  		  if(pwm < 1000)
	  		  {
	  			  pwm = 1000;
	  		  }
	  		  if(pwm > 2000)
	  		  {
	  			  pwm = 2000;
	  		  }
	  	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
	  	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm2);
	  		  previous_error = error_cascade;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pins : PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
