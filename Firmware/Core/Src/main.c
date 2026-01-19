/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "rc522.h"
#include "lcd1602_i2c.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CURRENT_HOURS      11
#define CURRENT_MINUTES    30
#define CURRENT_SECONDS    0
#define CURRENT_WEEKDAY    RTC_WEEKDAY_THURSDAY
#define CURRENT_DATE       15
#define CURRENT_MONTH      1
#define CURRENT_YEAR       26
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* --- 1. HARDWARE DEFINITIONS --- */
// Adjust pins to match your board
#define BTN_PORT    GPIOB
#define PIN_NEXT    BUTTON2_Pin  // Button to scroll options
#define PIN_OK      BUTTON3_Pin  // Button to select
#define PIN_BACK    BUTTON1_Pin  // Button to exit

#define LED_PORT    LED_STATUS_GPIO_Port
#define LED_PIN     LED_STATUS_Pin

#define BUZZ_PORT   BUZZER_GPIO_Port
#define BUZZ_PIN    BUZZER_Pin

// === MASTER KEY (ADMIN CARD) ===
// Change these 4 bytes to match your specific Admin Card!
const uint8_t MASTER_CARD_UID[4] = { 0xE3, 0x95, 0x45, 0xFA };

/* --- 2. THE ONE GLOBAL VARIABLE (STRUCT) --- */
typedef enum {
	STATE_IDLE_WAIT = 0,
	STATE_CHECK_CARD,
	STATE_ACCESS_GRANTED,
	STATE_ACCESS_DENIED,

	STATE_ADMIN_MENU,
	STATE_ADMIN_ADD_WAIT,
	STATE_ADMIN_ADD_WRITE,
	STATE_ADMIN_ADD_SUCCESS,
	STATE_ADMIN_DELETE_WAIT,
	STATE_ADMIN_DELETE_WRITE,
	STATE_ADMIN_DELETE_SUCCESS,

	STATE_ERROR
} state_t;

typedef struct {
	// -- FSM Logic --
	state_t state;           // Current State
	state_t next_state;      // Next State
	uint32_t entry_time;     // Timer for timeouts

	// -- System Data --
	uint8_t rfid_uid[4];     // The card we just scanned
	uint8_t menu_index;      // 0=Add, 1=Delete
	bool eeprom_result;      // Did the save work?
	bool lcd_needs_update;   // Do we need to repaint the screen?

	// -- Inputs (Flags) --
	bool card_detected;
	bool btn_next;
	bool btn_ok;
	bool btn_back;

} System_t;

// Global variable for everything
System_t sys;

// Drivers
RC522_Handle rc522;
LCD1602_I2C lcd;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
int set_time;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void system_init(void);
void read_inputs(void);
void fsm_update(void);
void fsm_output(void);
void fsm_enter_state(state_t new_state);
bool rfid_get_uid(uint8_t *uid_out);
bool check_master_card(uint8_t *uid);
void buzzer_beep(int times);
void led_heartbeat(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_RTC_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	system_init();
	fsm_enter_state(STATE_IDLE_WAIT);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (set_time) {
			set_time = 0;
			RTC_TimeTypeDef sTime = { 0 };
			sTime.Hours = CURRENT_HOURS;
			sTime.Minutes = CURRENT_MINUTES;
			sTime.Seconds = CURRENT_SECONDS;

			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

			RTC_DateTypeDef sDate = { 0 };
			sDate.WeekDay = CURRENT_WEEKDAY;
			sDate.Date = CURRENT_DATE;
			sDate.Month = CURRENT_MONTH;
			sDate.Year = CURRENT_YEAR;

			HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		read_inputs();
		fsm_update();
		fsm_output();
		led_heartbeat();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00201D2B;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */
	if (HAL_RTCEx_BKUPRead(&hrtc, 0) != 0x1234) //checking lost power rtc reset
			{
		/* USER CODE END Check_RTC_BKUP */

		/** Initialize RTC and set the Time and Date
		 */
		sTime.Hours = 0;
		sTime.Minutes = 0;
		sTime.Seconds = 0;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Month = RTC_MONTH_JANUARY;
		sDate.Date = 1;
		sDate.Year = 0;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN RTC_Init 2 */
		HAL_RTCEx_BKUPWrite(&hrtc, 0, 0x1234); //write Private "Key" data
	}
	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, EEPROM_WP_Pin | SPI1_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, BUZZER_Pin | LED_STATUS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_CS_Pin LD2_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : EEPROM_WP_Pin SPI1_RST_Pin */
	GPIO_InitStruct.Pin = EEPROM_WP_Pin | SPI1_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : BUZZER_Pin LED_STATUS_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin | LED_STATUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : BUTTON3_Pin BUTTON2_Pin BUTTON1_Pin */
	GPIO_InitStruct.Pin = BUTTON3_Pin | BUTTON2_Pin | BUTTON1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* -------------------------------------------------------------------------- */
/* 1. INPUT LAYER (Human & Sensor Interface)                                  */
/* -------------------------------------------------------------------------- */
/*
 * Purpose: Reads physical buttons and the RFID sensor.
 * Logic:
 * - Uses "Active Low" logic for buttons (0V = Pressed).
 * - Implements "Debouncing" (20ms wait) to ignore electrical noise.
 */
void read_inputs(void) {
	// Static flags to remember if a button is currently being held down
	static bool hold_next = false;
	static bool hold_ok = false;
	static bool hold_back = false;

	// --- BTN NEXT ---
	// 1. Check if signal is LOW (0V)
	if (HAL_GPIO_ReadPin(BTN_PORT, PIN_NEXT) == GPIO_PIN_RESET) {
		// 2. If it wasn't already held down...
		if (!hold_next) {
			HAL_Delay(20); // Wait for mechanical contacts to settle (Debounce)
			// 3. Check again. If still LOW, it's a real press.
			if (HAL_GPIO_ReadPin(BTN_PORT, PIN_NEXT) == GPIO_PIN_RESET) {
				sys.btn_next = true; // Set event flag for the FSM
				hold_next = true;    // Lock input until release
			}
		}
	} else {
		hold_next = false; // Button released (High/3.3V), unlock the logic
	}

	// --- BTN OK ---
	if (HAL_GPIO_ReadPin(BTN_PORT, PIN_OK) == GPIO_PIN_RESET) {
		if (!hold_ok) {
			HAL_Delay(20);
			if (HAL_GPIO_ReadPin(BTN_PORT, PIN_OK) == GPIO_PIN_RESET) {
				sys.btn_ok = true;
				hold_ok = true;
			}
		}
	} else {
		hold_ok = false;
	}

	// --- BTN BACK ---
	if (HAL_GPIO_ReadPin(BTN_PORT, PIN_BACK) == GPIO_PIN_RESET) {
		if (!hold_back) {
			HAL_Delay(20);
			if (HAL_GPIO_ReadPin(BTN_PORT, PIN_BACK) == GPIO_PIN_RESET) {
				sys.btn_back = true;
				hold_back = true;
			}
		}
	} else {
		hold_back = false;
	}

	// --- RFID POLLING ---
	// Purpose: Checks for cards periodically (every 50ms) instead of constantly.
	// Safety: We disable polling during WRITE states to prevent SPI bus conflicts.
	if (sys.state != STATE_ADMIN_ADD_WRITE
			&& sys.state != STATE_ADMIN_DELETE_WRITE) {
		static uint32_t last_rfid = 0;
		if (HAL_GetTick() - last_rfid > 50) {
			last_rfid = HAL_GetTick();

			// Ask RC522: "Is there a card field disruption?"
			RC522_Status status = RC522_IsNewCardPresent(&rc522);
			if (status == RC522_OK) {
				sys.card_detected = true;
			} else if (status == RC522_ERROR || status == RC522_TIMEOUT) {
				RC522_Reset(&rc522); // Reset chip if it gets stuck
			}
		}
	}
}

/* -------------------------------------------------------------------------- */
/* 2. LOGIC LAYER (The Finite State Machine)                                  */
/* -------------------------------------------------------------------------- */
/*
 * Purpose: Decides WHAT to do based on current state and inputs.
 * Logic:
 * - Checks flags set by read_inputs (sys.btn_next, sys.card_detected).
 * - Moves system from one state to another (e.g., IDLE -> CHECK_CARD).
 * - Handles timeouts (returning to IDLE after 2 seconds).
 */
void fsm_update(void) {
	sys.next_state = sys.state; // Default: Stay in current state

	switch (sys.state) {
	case STATE_IDLE_WAIT:
		// Waiting for a card...
		if (sys.card_detected) {
			// Try to read the unique ID (UID)
			if (rfid_get_uid(sys.rfid_uid)) {
				sys.next_state = STATE_CHECK_CARD;
			} else {
				// False alarm (noise or bad read)
				sys.card_detected = false;
				RC522_Reset(&rc522);
			}
		}
		break;

	case STATE_CHECK_CARD:
		// Decision: Is this Admin or User?
		if (check_master_card(sys.rfid_uid)) {
			sys.next_state = STATE_ADMIN_MENU;
		} else {
			// Check EEPROM database via I2C
			sys.eeprom_result = EEPROM_CheckUID(sys.rfid_uid);
			sys.next_state =
					sys.eeprom_result ?
							STATE_ACCESS_GRANTED : STATE_ACCESS_DENIED;
		}
		break;

	case STATE_ACCESS_GRANTED:
	case STATE_ACCESS_DENIED:
		// Show result for 2 seconds, then go home
		if ((HAL_GetTick() - sys.entry_time) > 2000)
			sys.next_state = STATE_IDLE_WAIT;
		break;

	case STATE_ADMIN_MENU:
		// Navigation Logic
		if (sys.btn_next) {
			sys.menu_index ^= 1;         // Toggle between 0 and 1
			sys.lcd_needs_update = true; // Tell Output Layer to redraw LCD
			sys.btn_next = false;        // Consume the event
		} else if (sys.btn_ok) {
			sys.next_state =
					(sys.menu_index == 0) ?
							STATE_ADMIN_ADD_WAIT : STATE_ADMIN_DELETE_WAIT;
			sys.btn_ok = false;
		} else if (sys.btn_back) {
			sys.next_state = STATE_IDLE_WAIT;
			sys.btn_back = false;
		}
		break;

	case STATE_ADMIN_ADD_WAIT:
		if (sys.card_detected)
			sys.next_state = STATE_ADMIN_ADD_WRITE;
		else if (sys.btn_back) {
			sys.next_state = STATE_ADMIN_MENU;
			sys.btn_back = false;
		}
		break;

	case STATE_ADMIN_ADD_WRITE:
		// Logic: Try to read UID. If fail, retry until timeout.
		if (rfid_get_uid(sys.rfid_uid)) {
			// Success! Prevent adding Master Card as a user.
			if (check_master_card(sys.rfid_uid)) {
				sys.next_state = STATE_ERROR;
			} else {
				// Save to EEPROM (I2C Write)
				sys.eeprom_result = (EEPROM_SaveUID(sys.rfid_uid) == 0);

				if (sys.eeprom_result) {
					sys.next_state = STATE_ADMIN_ADD_SUCCESS;
				} else {
					sys.next_state = STATE_ERROR; // EEPROM Full or Error
				}
			}
		} else {
			// Read failed. Keep trying for 2.5 seconds before giving up.
			if ((HAL_GetTick() - sys.entry_time) > 2500) {
				sys.next_state = STATE_ERROR;
			}
		}
		break;

	case STATE_ADMIN_ADD_SUCCESS:
		if ((HAL_GetTick() - sys.entry_time) > 1500)
			sys.next_state = STATE_ADMIN_MENU;
		break;

	case STATE_ADMIN_DELETE_WAIT:
		if (sys.card_detected)
			sys.next_state = STATE_ADMIN_DELETE_WRITE;
		else if (sys.btn_back) {
			sys.next_state = STATE_ADMIN_MENU;
			sys.btn_back = false;
		}
		break;

	case STATE_ADMIN_DELETE_WRITE:
		if (rfid_get_uid(sys.rfid_uid)) {
			if (check_master_card(sys.rfid_uid)) {
				sys.next_state = STATE_ERROR;
			} else {
				// Delete from EEPROM
				sys.eeprom_result = EEPROM_DeleteUID(sys.rfid_uid);

				if (sys.eeprom_result) {
					sys.next_state = STATE_ADMIN_DELETE_SUCCESS;
				} else {
					sys.next_state = STATE_ERROR; // User not found
				}
			}
		} else {
			// Retry timeout logic
			if ((HAL_GetTick() - sys.entry_time) > 2500) {
				sys.next_state = STATE_ERROR;
			}
		}
		break;

	case STATE_ADMIN_DELETE_SUCCESS:
		if ((HAL_GetTick() - sys.entry_time) > 1500)
			sys.next_state = STATE_ADMIN_MENU;
		break;

	case STATE_ERROR:
		// Universal error state. Wait for Back button OR Timeout.
		if (sys.btn_back || (HAL_GetTick() - sys.entry_time) > 2000) {
			sys.next_state = STATE_IDLE_WAIT;
			sys.btn_back = false;
		}
		break;
	}

	// Apply transition
	if (sys.next_state != sys.state)
		fsm_enter_state(sys.next_state);
}

/* -------------------------------------------------------------------------- */
/* 3. STATE TRANSITION HANDLER                                                */
/* -------------------------------------------------------------------------- */
/*
 * Purpose: Setup tasks that run ONLY ONCE when entering a new state.
 * Examples: Resetting timers, clearing flags, beeping the buzzer.
 */
void fsm_enter_state(state_t new_state) {
	sys.state = new_state;
	sys.entry_time = HAL_GetTick(); // Start the state timer
	sys.lcd_needs_update = true;    // Force LCD redraw

	// Reset inputs when entering "Wait" states to avoid accidental triggers
	if (new_state == STATE_IDLE_WAIT || new_state == STATE_ADMIN_ADD_WAIT
			|| new_state == STATE_ADMIN_DELETE_WAIT) {
		sys.card_detected = false;
		sys.btn_next = false;
		sys.btn_ok = false;
		RC522_Reset(&rc522);
	}

	// Audio Feedback
	switch (new_state) {
	case STATE_ACCESS_GRANTED:
		buzzer_beep(2);
		break;
	case STATE_ACCESS_DENIED:
		buzzer_beep(3);
		break;
	case STATE_ADMIN_MENU:
		buzzer_beep(1);
		break;
	case STATE_ADMIN_ADD_SUCCESS:
	case STATE_ADMIN_DELETE_SUCCESS:
		buzzer_beep(2);
		break;
	case STATE_ERROR:
		buzzer_beep(3);
		break;
	default:
		break;
	}
}

/* -------------------------------------------------------------------------- */
/* 4. OUTPUT LAYER (Visual Interface)                                         */
/* -------------------------------------------------------------------------- */
/*
 * Purpose: Manages the LCD display.
 * Strategy: Split into STATIC and DYNAMIC updates.
 * - Static: Text that never changes (e.g., "Access Denied"). Drawn ONCE.
 * - Dynamic: Text that changes (e.g., Clock). Drawn every second.
 * This prevents the screen from flickering.
 */
void fsm_output(void) {
	// --- A. STATIC UPDATES (Runs once per state change) ---
	if (sys.lcd_needs_update) {
		LCD_Clear(&lcd);
		LCD_SetCursor(&lcd, 0, 0);

		switch (sys.state) {
		case STATE_IDLE_WAIT:
			LCD_Print(&lcd, "\x03 Scan Card...");
			break;
		case STATE_ACCESS_GRANTED:
			LCD_Print(&lcd, "Access Granted");
			LCD_SetCursor(&lcd, 0, 1);
			LCD_Print(&lcd, "Welcome! \x01");
			break;
		case STATE_ACCESS_DENIED:
			LCD_Print(&lcd, "Access Denied");
			LCD_SetCursor(&lcd, 0, 1);
			LCD_Print(&lcd, "Invalid Card \x02");
			break;
		case STATE_ADMIN_MENU:
			LCD_Print(&lcd, "ADMIN MENU:");
			LCD_SetCursor(&lcd, 0, 1);
			if (sys.menu_index == 0)
				LCD_Print(&lcd, "> Add User");
			else
				LCD_Print(&lcd, "> Delete User");
			break;
		case STATE_ADMIN_ADD_WAIT:
			LCD_Print(&lcd, "ADD MODE");
			LCD_SetCursor(&lcd, 0, 1);
			LCD_Print(&lcd, "Scan New Card");
			break;
		case STATE_ADMIN_ADD_WRITE:
			LCD_Print(&lcd, "Saving...");
			break;
		case STATE_ADMIN_ADD_SUCCESS:
			LCD_Print(&lcd, "User Added! \x01");
			break;
		case STATE_ADMIN_DELETE_WAIT:
			LCD_Print(&lcd, "DELETE MODE");
			LCD_SetCursor(&lcd, 0, 1);
			LCD_Print(&lcd, "Scan to Erase");
			break;
		case STATE_ADMIN_DELETE_WRITE:
			LCD_Print(&lcd, "Erasing...");
			break;
		case STATE_ADMIN_DELETE_SUCCESS:
			LCD_Print(&lcd, "User Deleted! \x01");
			break;
		case STATE_ERROR:
			LCD_Print(&lcd, "Operation Failed");
			break;
		default:
			break;
		}
		sys.lcd_needs_update = false; // Mark as done
	}

	// --- B. DYNAMIC UPDATES (Clock) ---
	if (sys.state == STATE_IDLE_WAIT) {
		static uint8_t last_sec = 255;

		// Get Time from RTC (Real Time Clock) hardware
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		// Only send I2C command if the second has actually changed
		if (sTime.Seconds != last_sec) {
			last_sec = sTime.Seconds;

			char timeStr[25]; // Buffer large enough for text + icons
			// Format: DD/MM HH:MM:SS
			sprintf(timeStr, "%02d/%02d %02d:%02d:%02d", sDate.Date,
					sDate.Month, sTime.Hours, sTime.Minutes, sTime.Seconds);

			LCD_SetCursor(&lcd, 0, 1);
			LCD_Print(&lcd, timeStr);
		}
	}
}

/* -------------------------------------------------------------------------- */
/* 5. INITIALIZATION & HELPERS                                                */
/* -------------------------------------------------------------------------- */

void system_init(void) {
	// Initialize Drivers
	LCD_Init(&lcd, &hi2c1, 0x27, 16, 2);
	RC522_InitHandle(&rc522, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin,
	SPI1_RST_GPIO_Port, SPI1_RST_Pin);
	EEPROM_Init(&hi2c1);

	// Define Custom Icons (Bitmap)
	uint8_t Check[8] = { 0b00000, 0b00001, 0b00011, 0b10110, 0b11100, 0b01000,
			0b00000, 0b00000 };
	uint8_t Cross[8] = { 0b00000, 0b11011, 0b01110, 0b00100, 0b01110, 0b11011,
			0b00000, 0b00000 };
	uint8_t Heart[8] = { 0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100,
			0b00000, 0b00000 };

	// Upload Icons to LCD Memory (CGRAM)
	LCD_CreateCustomChar(&lcd, 1, Check); // Slot 1
	LCD_CreateCustomChar(&lcd, 2, Cross); // Slot 2
	LCD_CreateCustomChar(&lcd, 3, Heart); // Slot 3

	// Show Boot Screen
	LCD_Clear(&lcd);
	LCD_Print(&lcd, "System Boot");
	HAL_Delay(200);

	LCD_SetCursor(&lcd, 0, 1);
	LCD_Print(&lcd, "Init RFID...");
	HAL_Delay(800);

	// Initialize RFID Chip (Wait until success or timeout)
	uint32_t start = HAL_GetTick();
	while (RC522_Init(&rc522) != RC522_OK) {
		if (HAL_GetTick() - start > 1000) {
			LCD_Print(&lcd, "RFID Timeout");
			HAL_Delay(1000);
			break;
		}
	}
	LCD_Clear(&lcd);
}

// Wrapper to get RFID UID safely with timeout protection
bool rfid_get_uid(uint8_t *uid_out) {
	uint32_t t = HAL_GetTick();
	while (HAL_GetTick() - t < 50) {
		RC522_Status s = RC522_ReadCardUID(&rc522);
		if (s == RC522_OK) {
			memcpy(uid_out, rc522.uid.uidByte, 4);
			return true;
		}
		if (s != RC522_PENDING)
			break;
	}
	return false;
}

// Security Check: Is the scanned card the Admin Card?
bool check_master_card(uint8_t *uid) {
	return (memcmp(uid, MASTER_CARD_UID, 4) == 0);
}

// Simple Beep Function
void buzzer_beep(int times) {
	for (int i = 0; i < times; i++) {
		HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);
		if (times > 1)
			HAL_Delay(100);
	}
}

// Visual "Heartbeat" to prove the main loop is not frozen
void led_heartbeat(void) {
	static uint32_t last_blink = 0;
	if (HAL_GetTick() - last_blink > 500) {
		last_blink = HAL_GetTick();
		HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
