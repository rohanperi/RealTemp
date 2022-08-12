#include "main.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

// Variables for the TMP102 sensor - Use 8-bit address but last bit is to indicate read/write - hence left shift by 1
static const uint8_t TMP102_ADDR = 0x48 << 1;
static const uint8_t REG_TEMP = 0x00;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);

int __io_putchar(int ch);

void TempSensorTask(void *pvParameters);
void LedUpdateTask(void *pvParameters);
void reset_LED();
void led_Updater(float temp);

TaskHandle_t tempSensor_handle, ledUpdate_handle;

QueueHandle_t temperatureQueue;

typedef int TaskProfiler;

TaskProfiler TempSensorProfiler;
TaskProfiler LedUpdateProfiler;

int main(void)
{

	// Initializations
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_ETH_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_I2C1_Init();

	// Create a queue to transfer temperature data between tasks
	temperatureQueue = xQueueCreate(100, sizeof(float));

	// Task for reading from temperature sensor and putting data in the queue
	xTaskCreate(TempSensorTask,
				"Temperature Sensor Task",
				100,
				NULL,
				1,
				&tempSensor_handle);

	// Task to use temperature values from the queue and update LEDs
	xTaskCreate(LedUpdateTask,
				"Led Update Task",
				100,
				NULL,
				2,
				&ledUpdate_handle);

	vTaskStartScheduler();

  while (1)
  {

  }
}

void TempSensorTask(void *pvParameters) {
	HAL_StatusTypeDef ret;
	uint8_t buf[12];
	int16_t val;
	float temp_c;
	BaseType_t qStatus;

	while(1) {
		TempSensorProfiler++;

		// First byte of buffer to location of TMP102's temperature register
		buf[0] = REG_TEMP;

		// Tell TMP102 we want to read
		ret = HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);

		if (ret != HAL_OK) {
		  printf("Error Tx\r\n");
		} else {

			//Read from the TMP102 sensor
			ret = HAL_I2C_Master_Receive(&hi2c1, TMP102_ADDR, buf, 2, HAL_MAX_DELAY);

		    if (ret != HAL_OK) {
		    	printf("Error Rx\r\n");
		    } else {

			    // Combine the bytes - constructs our 12-bit value
			    val = ((int16_t)buf[0] << 4 | (buf[1] >> 4));

			    // Convert to 2's complement, since temperature can be negative
			    if (val > 0x7FF) {
				    val |= 0xF00;
			    }

			    // Convert to float temperature value as required from datasheet (Celsius)
			    temp_c = val * 0.0625;
		    }
		}

		printf("Temperature(C): %.2f \r\n", temp_c);

		// Send the temperature value into our queue for LedUpdateTask
		qStatus = xQueueSend(temperatureQueue, &temp_c, 0);

		vTaskDelay(1);

		if(qStatus != pdPASS) {
			printf("Error: Temperature data could not be sent ... \r\n");
		}
	}
}

void LedUpdateTask(void *pvParameters) {

	float temp_received;

	// Amount of time to wait to receive from front of queue
	const TickType_t wait_time = pdMS_TO_TICKS(100);
	BaseType_t qStatus;

	while(1) {
		LedUpdateProfiler++;

		// Retrieve the temperature values from the queue
		qStatus = xQueueReceive(temperatureQueue, &temp_received, wait_time);

		vTaskDelay(1);

		if(qStatus == pdPASS) {

			// Reset then update LEDs
			reset_LED();
			led_Updater(temp_received);

		} else {
			printf("Error! Could not receive temperature \r\n");
		}
	}
}

void reset_LED() {
	// Reset all GPIO pins wired to LEDs
	HAL_GPIO_WritePin(TempLed1_GPIO_Port, TempLed1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TempLed2_GPIO_Port, TempLed2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TempLed3_GPIO_Port, TempLed3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TempLed4_GPIO_Port, TempLed4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TempLed5_GPIO_Port, TempLed5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TempLed6_GPIO_Port, TempLed6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TempLed7_GPIO_Port, TempLed7_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TempLed8_GPIO_Port, TempLed8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TempLed9_GPIO_Port, TempLed9_Pin, GPIO_PIN_RESET);
}

void led_Updater(float temp) {
	// Use a range from 23.5 - 25.0 degrees Celsius to show variations in temperature - adapted to an average room temperature
	if(temp < 23.5) {
		HAL_GPIO_WritePin(TempLed9_GPIO_Port, TempLed9_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TempLed8_GPIO_Port, TempLed8_Pin, GPIO_PIN_SET);
	} else if(temp >= 23.5 && temp < 24.0) {
		HAL_GPIO_WritePin(TempLed7_GPIO_Port, TempLed7_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TempLed6_GPIO_Port, TempLed6_Pin, GPIO_PIN_SET);
	} else if(temp >= 24.0 && temp < 24.5) {
		HAL_GPIO_WritePin(TempLed5_GPIO_Port, TempLed5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TempLed4_GPIO_Port, TempLed4_Pin, GPIO_PIN_SET);
	} else if(temp >= 24.5 && temp < 25.0) {
		HAL_GPIO_WritePin(TempLed3_GPIO_Port, TempLed3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TempLed2_GPIO_Port, TempLed2_Pin, GPIO_PIN_SET);
	} else if(temp > 25.0) {
		HAL_GPIO_WritePin(TempLed1_GPIO_Port, TempLed1_Pin, GPIO_PIN_SET);
	}
}

// Custom function for writing to UART
int uart3_write(int ch) {
	 // Check that transmit data register is empty  in the Interrupt and Service register
	while(!(USART3->ISR & USART_ISR_TXE));
	// Write our data to the transmit data register
	USART3->TDR = (ch & 0xFF);
	return ch;
}

// Wires printf to use UART
int __io_putchar(int ch) {
	uart3_write(ch);
	return ch;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TempLed4_Pin|TempLed5_Pin|TempLed6_Pin|TempLed7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TempLed2_Pin|TempLed3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TempLed1_GPIO_Port, TempLed1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TempLed8_Pin|TempLed9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TempLed4_Pin TempLed5_Pin TempLed6_Pin TempLed7_Pin */
  GPIO_InitStruct.Pin = TempLed4_Pin|TempLed5_Pin|TempLed6_Pin|TempLed7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : TempLed2_Pin TempLed3_Pin */
  GPIO_InitStruct.Pin = TempLed2_Pin|TempLed3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TempLed1_Pin */
  GPIO_InitStruct.Pin = TempLed1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TempLed1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TempLed8_Pin TempLed9_Pin */
  GPIO_InitStruct.Pin = TempLed8_Pin|TempLed9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
