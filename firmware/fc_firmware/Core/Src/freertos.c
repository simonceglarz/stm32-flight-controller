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
#include "usart.h"
#include "roll_ctrl.h"
#include "hil_proto.h"   /* HIL message format from core/ */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* One telemetry sample, passed from the control task to the telemetry task */
typedef struct {
	uint32_t tick;	/*ms since boot */
	float az_g;		/*accel Z, g */
	float gx_dps;	/*raw gyro X, deg/s */
	float acc_deg;	/*accel-only angle, deg */
	float angle_deg;/*filtered angle, deg */
	float cmd;		/* controller output, -1..1 (no motors yet) */
} telem_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_FLAG		0x01u	/*thread flag set by the data-ready interrupt*/
#define IMU_ODR_HZ		1608.0f	/*IMU output rate = loop rate (REQ-01) */
#define TELEM_DIVIDER	166u	/*send 1 of every 166 samples -> 10Hz telemetry*/
#define DRDY_TIMEOUT_MS	5u		/*no data-ready for 5ms = IMU fault (seed for REQ-02*/

/* 1 = HIL mode: PC sends sensor values, board replies with motor command.
   0 = normal mode: real IMU drives the loop. Rebuild + reflash after changing. */
#define HIL_MODE        1
#define HIL_DT_S        (1.0f / 1608.0f)   /* simulated step, s: matches the real loop rate (622 us) */
#define HIL_TX_TIMEOUT  10u                /* ms: max wait to send a 13-byte reply (~1.1 ms at 115200) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static osMessageQueueId_t telemQueue;	/*mailbox between control and telemetry tasks */
static osMessageQueueId_t rxQueue;      /* bytes received from the PC, waiting to be parsed */
static uint8_t rx_byte;                 /* where the UART drops each received byte */
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
static void TelemetryTask(void *argument);
static void hil_loop(void);                  /* HIL main loop: never returns */
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
	(void)xTask; (void)pcTaskName;
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
	/*16 slots; if the telemetry task falls behind, new samples are dropped
	instead of the control task waiting */
	telemQueue = osMessageQueueNew(16, sizeof(telem_t), NULL);
	/* 256 one-byte slots: room for 12 SENSOR frames, far more than lock-step needs */
	rxQueue = osMessageQueueNew(256, sizeof(uint8_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  const osThreadAttr_t telemAttr = {
		  .name = "telem",
		  .stack_size = 512*4,			/*bytes; prinf with floats needs more room */
		  .priority = osPriorityLow,	/*below the control task; printing never delays control */
  };
  osThreadNew(TelemetryTask, NULL, &telemAttr);
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

	(void)argument;

#if HIL_MODE
  hil_loop();          /* PC drives everything; IMU, filter and telemetry are skipped */
#endif

    /* ---- Otherwise, Normal mode (unchanged below) ---- */
	int rc = lsm6dsox_init();		/*bring up the IMU; 0 = success*/
	printf("IMU init: %d\r\n", rc);	/*report which step failed, if any*/
	  /* Factory-trimmed oscillator deviation for this specific chip (DS12814, reg 0x63).
	     Signed 8-bit: each count = [scale from datasheet] % of nominal ODR. */
	  int8_t freq_fine = lsm6dsox_read_freq_fine();
	  printf("INTERNAL_FREQ_FINE: %d (~%.2f%% vs nominal)\r\n",
	         freq_fine, (double)(freq_fine * 0.15f));   /* 0.15 %/LSB, DS12814 Table 174 */


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

	/* Calibration reads ran while data-ready pulses were firing, so flags
     are already set. Clear them so the first loop waits for a fresh sample. */
	  osThreadFlagsClear(IMU_FLAG);

	  const float dt = 1.0f / IMU_ODR_HZ;   /* 0.6 ms nominal sample period */
	  uint32_t n = 0;
	  uint32_t drdy_timeouts = 0;

	  roll_ctrl_t ctrl;
	  roll_ctrl_init(&ctrl, &ROLL_GAINS_DEFAULT);	/*gains from SIL tuning */

	  for (;;) {
		/* Sleep until the interrupt says a new sample is ready. The task uses
		   no CPU while waiting; the IMU's clock sets the loop rate. */
		uint32_t f = osThreadFlagsWait(IMU_FLAG, osFlagsWaitAny, DRDY_TIMEOUT_MS);
		if (f & osFlagsError) {             /* timed out: no sample in 5 ms */
		  drdy_timeouts++;                  /* later: REQ-02 -> zero motors, FAULT state */
		  continue;
		}

    HAL_GPIO_WritePin(LOOP_PIN_GPIO_Port, LOOP_PIN_Pin, GPIO_PIN_SET);   /* loop start */

    uint8_t raw[12];
    if (lsm6dsox_read_raw(raw) == 0) {
      imu_sample_t s;
      imu_convert(raw, &s);
      float acc_angle = cf_accel_angle_deg(s.accel_g[1], s.accel_g[2]);
      float angle = cf_update(&roll, s.gyro_dps[0], acc_angle, dt);

      /* Inner loop needs the bias-correcct rate (the filter corrects internally,
       *  but the raw s.gyro_dps[0] still contains the ~0.5 dps bias */
      float rate_dps = s.gyro_dps[0] - roll.gyro_bias_dps;
      /* Hold level, Output drives nothing yet; it is logged so we can check sign and size */
      float cmd = roll_ctrl_step(&ctrl, 0.0f, angle, rate_dps, dt);

      if (++n % TELEM_DIVIDER == 0) {
        telem_t t = { osKernelGetTickCount(), s.accel_g[2], s.gyro_dps[0], acc_angle, angle, cmd };
        osMessageQueuePut(telemQueue, &t, 0, 0);   /* timeout 0: never wait here */
      }
    }

    HAL_GPIO_WritePin(LOOP_PIN_GPIO_Port, LOOP_PIN_Pin, GPIO_PIN_RESET); /* loop end */
  }

  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Low-priority task: the only place that prints. A ~40-character line takes
   ~3.5 ms at 115200 baud; running here, it gets interrupted by the control
   task instead of blocking it. */
static void TelemetryTask(void *argument)
{
  (void)argument;
  telem_t t;
  for (;;) {
    if (osMessageQueueGet(telemQueue, &t, NULL, osWaitForever) == osOK) {
    	char line[64];                                    /* one CSV line, ~40 chars */
    	int len = snprintf(line, sizeof line, "%lu,%.3f,%.2f,%.2f,%.2f,%3f\r\n",
    	                   t.tick, (double)t.az_g, (double)t.gx_dps,
    	                   (double)t.acc_deg, (double)t.angle_deg, (double)t.cmd);
    	if (len > 0) {
    	  /* No newlib stdout lock held while sending, so the control task
    	     can preempt this at any point. */
    	  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, HAL_MAX_DELAY);
    	}
    }
  }
}

/* Runs in interrupt context on each PC7 rising edge (IMU data-ready).
   Does the minimum: wake the control task, then return. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_INT1_Pin && defaultTaskHandle != NULL) {
    osThreadFlagsSet(defaultTaskHandle, IMU_FLAG);   /* safe from an interrupt */
  }
}

/* ---------------------------------------------------------------------------
 * HIL main loop. Replaces the IMU loop when HIL_MODE is 1.
 * Each pass: collect bytes until a complete SENSOR message arrives, run the
 * controller on its values, and send back a CMD message with the same number.
 * ------------------------------------------------------------------------- */
static void hil_loop(void)
{
  roll_ctrl_t ctrl;                          /* the same controller as in normal mode */
  roll_ctrl_init(&ctrl, &ROLL_GAINS_DEFAULT);

  hil_parser_t parser;                       /* rebuilds messages from single bytes */
  hil_parser_init(&parser);

  uint8_t out[HIL_MAX_FRAME];                /* reply bytes are built here */

  /* Ask the UART for the first byte. From then on the callback re-arms itself. */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  for (;;) {
    /* 1. Wait (sleeping, no CPU used) until the interrupt has queued a byte */
    uint8_t b;
    if (osMessageQueueGet(rxQueue, &b, NULL, osWaitForever) != osOK) {
      continue;
    }

    /* 2. Give the byte to the parser. Only act once a whole, undamaged
          SENSOR message has arrived; every other byte just returns here. */
    if (hil_parser_feed(&parser, b) != HIL_TYPE_SENSOR) {
      continue;
    }
    hil_sensor_t s;
    if (hil_decode_sensor(&parser, &s) != 0) {
      continue;                              /* wrong size: ignore */
    }

    HAL_GPIO_WritePin(LOOP_PIN_GPIO_Port, LOOP_PIN_Pin, GPIO_PIN_SET);   /* timing: start */

    /* 3. Message #0 = the PC is starting a new run, so clear the controller's memory */
    if (s.seq == 0) {
      roll_ctrl_reset(&ctrl);
    }

    /* 4. Exactly the call the real loop makes, with values from the PC */
    hil_cmd_t c = {
      .seq    = s.seq,                       /* echo the number so the PC can match the reply */
      .cmd    = roll_ctrl_step(&ctrl, s.angle_sp_deg, s.angle_deg, s.rate_dps, HIL_DT_S),
      .status = HIL_STATUS_OK,
    };

    /* 5. Package and send the reply */
    size_t n = hil_encode_cmd(&c, out);

    HAL_GPIO_WritePin(LOOP_PIN_GPIO_Port, LOOP_PIN_Pin, GPIO_PIN_RESET); /* timing: end */

    HAL_UART_Transmit(&huart2, out, (uint16_t)n, HIL_TX_TIMEOUT);
  }
}

/* ---------------------------------------------------------------------------
 * Called by HAL (inside the USART2 interrupt) each time one byte has arrived.
 * Keep it tiny: store the byte, re-arm, return. The task does the real work.
 * ------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    osMessageQueuePut(rxQueue, &rx_byte, 0, 0);   /* timeout 0: if the queue is ever full, drop
                                                     the byte rather than wait (interrupts can't
                                                     wait); the CRC will reject that frame */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);    /* ask for the next byte */
  }
}

/* ---------------------------------------------------------------------------
 * Called by HAL if the UART reports a receive error (for example "overrun":
 * a new byte arrived before the previous one was collected). HAL stops
 * receiving after an error, so restart it, or the board would go deaf.
 * ------------------------------------------------------------------------- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}
/* USER CODE END Application */

