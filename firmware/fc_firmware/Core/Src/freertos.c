/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lsm6dsox.h"
#include "imu_convert.h"
#include "comp_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

	/*---One-time setup*/
	int rc = lsm6dsox_init();		/*bring up the IMU; 0 = success*/
	printf("IMU init: %d\r\n", rc);	/*report which step failed, if any*/

	if(rc !=0)
	{
		/* Init failed: stop here and blink fast (10Hz) so failure is visible
		 * even without a terminal open. The task never continues*/
		for (;;)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			osDelay(100);
		}
	}

	  comp_filter_t roll;
	  cf_init(&roll, 0.47f);                     /* tau = 0.47 s */

	  /* 1. Let the gyro settle after power-on. Its first outputs are not
	        valid, and calibrating on them gives a wrong bias. */
	  osDelay(200);                              /* 200 ms, generous */
	  for (int i = 0; i < 20; i++) {             /* throw away 20 samples (~200 ms) */
	    uint8_t junk[12];
	    osDelay(10);
	    (void)lsm6dsox_read_raw(junk);           /* (void) = result deliberately ignored */
	  }

	  /* 2. Measure the bias: 100 samples, 10 ms apart = 1 s. Board must be still. */
	  float gx_samples[100];
	  float gmin = 1e9f, gmax = -1e9f;           /* smallest and largest sample seen */
	  for (int i = 0; i < 100; i++) {
	    uint8_t raw_cal[12];
	    imu_sample_t cal;
	    osDelay(10);
	    if (lsm6dsox_read_raw(raw_cal) != 0) { i--; continue; }  /* retry a failed read */
	    imu_convert(raw_cal, &cal);
	    gx_samples[i] = cal.gyro_dps[0];
	    if (gx_samples[i] < gmin) gmin = gx_samples[i];
	    if (gx_samples[i] > gmax) gmax = gx_samples[i];
	  }
	  cf_calibrate_bias(&roll, gx_samples, 100);

	  /* 3. Print what calibration saw, so a bad calibration is visible. */
	  printf("gyro X bias: %.3f dps\r\n", (double)roll.gyro_bias_dps);
	  printf("cal spread: %.2f to %.2f dps\r\n", (double)gmin, (double)gmax);
	  printf("first/last cal sample: %.2f / %.2f dps\r\n",
	         (double)gx_samples[0], (double)gx_samples[99]);

	/*OSKernelGetTickCount() = FreeRTOS's millisecond counter since start-up
	 'next' is the absolute time (in ticks) when the last loop should run. */
	uint32_t next = osKernelGetTickCount();
	uint32_t n = 0;			//loop counter, used to print less often

	/*---100Hz Loop---*/
	for(;;)
	{
		next +=10;		//schedule the next run 10ms later

		/*Sleep until the absolute time 'next'. Unlike osDely(10), which waits 10ms
		 * AFTER this loop;s work, this keeps a stead 10ms period no how long the
		 * work took, so the loop rate doesn't drift */
		osDelayUntil(next);

		/*Read the 12 raw bytes. On a bus error, skip this sample and try next time. */
		uint8_t raw[12];
		if (lsm6dsox_read_raw(raw) != 0) {
			printf("read error\r\n");
			continue;
		}

		/* Convert raw counts to physical units (test on the PC; test_imu_convert. */
		imu_sample_t s;
		imu_convert(raw, &s);

		/* Angle from gravity; how gravity splits between the Y and  axes as the board
		 * rolls around its X axis. Noisy, but doesnt drift. */
		float acc_angle = cf_accel_angle_deg(s.accel_g[1], s.accel_g[2]);

		/* Fuse: integrate the X gyro (smooth,drifts) and pull gently toward
		 * accel angle (noisy, no drift). dt = 0.01s because the loop runs every 10ms. */
		float angle = cf_update(&roll, s.gyro_dps[0], acc_angle, 0.01f);

		/* Print only every 10th loop (10Hz). At 115200 baud one line takes ~5ms
		 * to send, so printing every loop would eat half of each 10ms period. */
		if (++n % 10 ==0) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);		/*slow blink = loop is alive */
			printf("%lu,%.3f,%.2f,%.2f,%.2f\r\n",
			       osKernelGetTickCount(),           /* time in ms since start-up            */
			       (double)s.accel_g[2],             /* az, g                                */
			       (double)s.gyro_dps[0],            /* gx, deg/s                            */
			       (double)acc_angle,                /* accelerometer-only angle, deg        */
			       (double)angle);                   /* filtered angle, deg                  */
		}
	}

  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

