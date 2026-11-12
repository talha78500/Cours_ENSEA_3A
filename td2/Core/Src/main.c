/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} color_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_BLINK_PERIOD   250      // ms

/* NeoPixel : TIM2 a 170 MHz, Prescaler = 0, Counter Period (ARR) = 212
 * -> periode d'un bit = 213 ticks = 1,25 us
 * -> bit 0 : T0H ~ 0,4 us = 68 ticks
 * -> bit 1 : T1H ~ 0,8 us = 136 ticks
 * A adapter si tes valeurs du TD NeoPixel sont differentes. */
#define CCR_BIT0           68
#define CCR_BIT1           136

#define NB_LED             1
#define NB_CHANNELS        3        // G, R, B
#define NB_BITS_PER_CHAN   8
#define NB_BITS            (NB_LED * NB_CHANNELS * NB_BITS_PER_CHAN)
#define BUFFER_SIZE        (NB_BITS + 1)   // +1 : zero final (Q11), sortie a l'etat bas

#define HUE_STEP           5        // degres de teinte par impulsion d'encodeur
#define HSV_S              250
#define HSV_V              50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Drapeaux ecrits en interruption, lus dans la boucle -> volatile */
static volatile bool     sw1_pressed = false;
static volatile bool     sw2_pressed = false;
static volatile bool     np_busy     = false;
static volatile uint32_t np_end_tick = 0;

/* Buffer DMA : global, il doit rester valide pendant tout le transfert */
static uint8_t np_buffer[BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------- Conversion HSV -> RGB (fournie dans le cours) ---------- */
static void hsv_to_rgb(int h, int s, int v, uint8_t *r, uint8_t *g, uint8_t *b) {
  h = ((h % 360) + 360) % 360;
  if (s <= 0) {
    *r = *g = *b = (uint8_t)v;
    return;
  }

  int region = h / 60;
  int rem = h % 60;
  int p = (v * (255 - s)) / 255;
  int q = (v * (255 - (s * rem) / 60)) / 255;
  int t = (v * (255 - (s * (60 - rem)) / 60)) / 255;

  switch (region) {
    case 0:  *r = (uint8_t)v; *g = (uint8_t)t; *b = (uint8_t)p; break;
    case 1:  *r = (uint8_t)q; *g = (uint8_t)v; *b = (uint8_t)p; break;
    case 2:  *r = (uint8_t)p; *g = (uint8_t)v; *b = (uint8_t)t; break;
    case 3:  *r = (uint8_t)p; *g = (uint8_t)q; *b = (uint8_t)v; break;
    case 4:  *r = (uint8_t)t; *g = (uint8_t)p; *b = (uint8_t)v; break;
    default: *r = (uint8_t)v; *g = (uint8_t)p; *b = (uint8_t)q; break;
  }
}

/* ---------- Couleur -> buffer de valeurs CCR ---------- */
/* Ordre WS2812B : G, R, B, bit de poids fort (MSB) en premier */
static void color_to_buffer(color_t *color, uint8_t *buffer) {
  uint8_t channels[NB_CHANNELS] = { color->g, color->r, color->b };
  int idx = 0;

  for (int c = 0; c < NB_CHANNELS; c++) {
    for (int bit = NB_BITS_PER_CHAN - 1; bit >= 0; bit--) {
      buffer[idx++] = (channels[c] & (1 << bit)) ? CCR_BIT1 : CCR_BIT0;
    }
  }
  buffer[idx] = 0;   // zero final : la sortie reste a l'etat bas apres la trame
}

/* ---------- Foreground : interruptions ---------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
    case BTN_SW1_Pin: sw1_pressed = true; break;
    case BTN_SW2_Pin: sw2_pressed = true; break;
    default: break;
  }
}

/* Appelee a la fin du transfert DMA TIM2_CH1 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    np_end_tick = HAL_GetTick();
    np_busy = false;
  }
}

/* ---------- NeoPixel ---------- */
/* Pret si aucun transfert en cours ET temps de reset ecoule.
 * >= 2 ticks garantit au moins 1 ms reelle (resolution de HAL_GetTick). */
static bool np_ready(void) {
  return !np_busy && ((HAL_GetTick() - np_end_tick) >= 2);
}

static void np_send(color_t *c) {
  color_to_buffer(c, np_buffer);
  np_busy = true;
  if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1,
                            (uint32_t *)np_buffer, BUFFER_SIZE) != HAL_OK) {
    np_busy = false;   // l'envoi n'a pas demarre
  }
}

/* ---------- UART (Tera Term) ---------- */
static void uart_print(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 100);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  int16_t  last_enc    = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  bool     need_update = true;     // envoie une premiere couleur au demarrage (rouge si enc = 0)
  uint32_t last_toggle = HAL_GetTick();
  char     msg[64];

  uart_print("\r\n--- Encodeur -> NeoPixel ---\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 1. Clignotement LED1, non bloquant */
    if ((HAL_GetTick() - last_toggle) >= LED_BLINK_PERIOD) {
      last_toggle += LED_BLINK_PERIOD;
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

    /* 2. Boutons (drapeaux leves en interruption) */
    if (sw1_pressed) {
      sw1_pressed = false;
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
    if (sw2_pressed) {
      sw2_pressed = false;
      HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    }

    /* 3. Lecture de l'encodeur */
    int16_t enc = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    if (enc != last_enc) {
      last_enc = enc;
      need_update = true;
    }

    /* 4. Envoi a la NeoPixel seulement si changement ET pret */
    if (need_update && np_ready()) {
      need_update = false;

      int hue = enc * HUE_STEP;
      color_t c;
      hsv_to_rgb(hue, HSV_S, HSV_V, &c.r, &c.g, &c.b);
      np_send(&c);

      snprintf(msg, sizeof(msg), "enc = %6d  hue = %3d  R=%3u G=%3u B=%3u\r\n",
               enc, ((hue % 360) + 360) % 360, c.r, c.g, c.b);
      uart_print(msg);
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
#ifdef USE_FULL_ASSERT
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
