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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "adxl312_driver.h"
#include "accel_calibration.h"
#include "lis2mdl_driver.h"
#include "mag_calibration.h"
#include "adxl356_driver.h"
#include "adxl356_calibration.h"


//#include "spi.h"
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef hlpuart1;  // or huart2 if you're using USART2
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

//ADXL312 WORKING CODE START!!!
/*
#define ADXL312_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define ADXL312_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)


uint8_t ADXL312_ReadRegister(uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };  // MSB=1 for read
    uint8_t rx[2] = {0};

    ADXL312_CS_LOW();

    // Use a full delay instead of __NOPs
    HAL_Delay(1);  // Give 1 ms delay after CS goes low

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

    ADXL312_CS_HIGH();

    return rx[1];
}

void ADXL312_ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t tx[7] = { 0x32 | 0xC0 }; // Burst read starting at 0x32, multibyte
    uint8_t rx[7] = {0};

    ADXL312_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, HAL_MAX_DELAY);
    ADXL312_CS_HIGH();

    *x = (int16_t)((rx[2] << 8) | rx[1]);
    *y = (int16_t)((rx[4] << 8) | rx[3]);
    *z = (int16_t)((rx[6] << 8) | rx[5]);
}

void ADXL312_Init(void)
  {
      // Put the device into measurement mode
      uint8_t tx[2];
      tx[0] = 0x2D;         // POWER_CTL register
      tx[1] = 0x08;         // Set Measure bit (D3 = 1)

      ADXL312_CS_LOW();
      HAL_Delay(1);
      HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
      ADXL312_CS_HIGH();
      HAL_Delay(10);
  }
*/
//ADXL312 WORKING CODE END

//#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
#define LSM303AH_ACC_ADDR (0x1D << 1) // 0x3A write address
#define LSM303AH_WHO_AM_I 0x0F
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t count=0;
uint8_t tx_buffer[27]="Welcome! \n\r";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/*
void LSM303AH_ReadWHOAMI(void);
uint16_t Read_ADC_Channel(void);
*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/*
int _write(int file, char *ptr, int len)
{
	for (int i=0; i<len;i++){
		ITM_SendChar(*ptr++);
	}
	return len;
}

void LSM303AH_ReadWHOAMI(void) {
    uint8_t whoami = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, LSM303AH_ACC_ADDR, LSM303AH_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoami, 1, HAL_MAX_DELAY) == HAL_OK) {
        printf("WHO_AM_I = 0x%02X\r\n", whoami);
    } else {
        printf("I2C Read Failed\r\n");
    }
}
*/

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
  MX_USART2_UART_Init();
  MX_LPUART1_UART_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE BEGIN 2 */


//TESTING STEVALMKI173V1 CODE START
/*
  uint8_t acc_whoami = 0;
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(&hi2c2, 0x1D << 1, 0x0F, I2C_MEMADD_SIZE_8BIT, &acc_whoami, 1, HAL_MAX_DELAY);

  if (status == HAL_OK)
  {
      printf("Accelerometer WHO_AM_I = 0x%02X\r\n", acc_whoami);  // Expect 0x43
  }
  else
  {
      printf("Accelerometer WHO_AM_I read failed (status: %d)\r\n", status);
  }


  status = HAL_I2C_Mem_Read(&hi2c2, 0x1D << 1, 0x0F, I2C_MEMADD_SIZE_8BIT, &acc_whoami, 1, HAL_MAX_DELAY);

  if (status == HAL_OK)
  {
      printf("Accelerometer WHO_AM_I = 0x%02X\r\n", acc_whoami);  // Expect 0x43
  }
  else
  {
      printf("Accelerometer WHO_AM_I read failed (status: %d)\r\n", status);
  }
*/
/*
  uint8_t mag_id = 0;
  HAL_I2C_Mem_Read(&hi2c1, 0x1E << 1, 0x4F, 1, &mag_id, 1, 100);
  printf("Magnetometer WHO_AM_I = 0x%02X\r\n", mag_id);

  printf("Final I2C2 scan:\r\n");
  for (uint8_t addr = 1; addr < 127; addr++)
  {
      if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 50) == HAL_OK)
      {
          printf("Device found at 0x%02X\r\n", addr << 1);
      }
  }


  printf("Scanning I2C2 bus...\r\n");
*/

  /*
  HAL_Delay(50);
  printf("Scanning I2C2 bus...\r\n");

  for (uint8_t addr = 1; addr < 127; addr++) {
      HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 50);
      if (result == HAL_OK) {
          printf("Found device at 0x%02X\r\n", addr << 1);
          HAL_Delay(100);
      }
  }

  HAL_Delay(50);
  for (uint8_t addr = 1; addr < 127; addr++)
  {
      HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 50);
      if (result == HAL_OK)
      {
          printf("Found I2C device at address 0x%02X\r\n", addr << 1);
          HAL_Delay(100);  // prevent terminal flooding
      }
  }
  */
  /*
  HAL_Delay(50);



  */

/*
  uint8_t acc_whoami = 0;
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(&hi2c1, 0x1D << 1, 0x0F, I2C_MEMADD_SIZE_8BIT,
		  &acc_whoami, 1, HAL_MAX_DELAY);

  if (status == HAL_OK)
  {
      printf("LSM303AH Accelerometer WHO_AM_I = 0x%02X\r\n", acc_whoami);
  }
  else
  {
      printf("LSM303AH WHO_AM_I read failed (status: %d)\r\n", status);
  }
*/

//WORKING STEVALMKI181V1 CODE START

  // === Calibrate LIS2MDL Zero-Offset ===
/*
  int16_t offset_x = 0, offset_y = 0, offset_z = 0;
  #define CALIB_SAMPLES 50
  int32_t sum_x = 0, sum_y = 0, sum_z = 0;

  for (int i = 0; i < CALIB_SAMPLES; i++)
  {
      uint8_t data[6] = {0};
      HAL_I2C_Mem_Read(&hi2c1, 0x3C, 0x68, 1, data, 6, HAL_MAX_DELAY);
      sum_x += (int16_t)(data[1] << 8 | data[0]);
      sum_y += (int16_t)(data[3] << 8 | data[2]);
      sum_z += (int16_t)(data[5] << 8 | data[4]);
      HAL_Delay(20);
  }

  offset_x = sum_x / CALIB_SAMPLES;
  offset_y = sum_y / CALIB_SAMPLES;
  offset_z = sum_z / CALIB_SAMPLES;

  printf("Offset calibrated: X=%d Y=%d Z=%d\r\n", offset_x, offset_y, offset_z);
*/
//WORKING STEVALMKI181V1 CODE END


//ADXL312 WORKING CODE START!!!
  /*
  ADXL312_CS_HIGH();  // Keep CS high when idle
  HAL_Delay(10);      // Give the sensor a moment to stabilize

  // Try reading the Device ID register (should return 0xE5)
  uint8_t dev_id = ADXL312_ReadRegister(0x00);
  printf("ADXL312 Device ID = 0x%02X\r\n", dev_id);


  ADXL312_Init();  // <<-- This is critical

  ADXL312_Init();
// Run bias calibration (blocks until complete)
calibrateAccelerometer();
while (!accelCalibrationComplete);

*/
//ADXL312 WORKING CODE END!!!

  //ADXL312 GPT Architecture Working Code Begin
  //calibrate the accelerometer
/*
  ADXL312_Init();

  // Repeatedly sample until calibration completes
  while (!accelCalibrationComplete) {
      calibrateAccelerometer();
  }

*/
//ADXL312 GPT Architecture Working Code End

  //lis2mdl GPT ARCHITECTURE TESTING CODE BEGIN
  //calibrate the magnetometer
/*
  LIS2MDL_Init();
  while (!magCalibrationComplete) {
      calibrateMagnetometer();
  }
*/
  //LIS2MDL GPT ARCHITECTURE TESTING CODE END

  //adxl356 GPT ARCHITECTURE WORKING CODE START
/*
  ADXL356_Init();
  while (!accel356CalibrationComplete) {
      calibrateAccel356();
  }
*/
  //ADXL356 GPT ARCHITECTURE WORKING CODE END

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //HAL_ADC_Start(&hadc1); //CRUCIAL FOR ADXL356 COM
  while (1)
  {

	  //ADXL356 GPT ARCHITECTURE WORKING CODE START
/*
	  float rxg, ryg, rzg, cxg, cyg, czg;

	  // 1) get raw in g
	  ADXL356_ReadAccelG(&rxg, &ryg, &rzg);
	  // 2) apply bias correction
	  getCalibratedAccel356(rxg, ryg, rzg, &cxg, &cyg, &czg);
	  // 3) print calibrated values
	  printf("X: %.2f g, Y: %.2f g, Z: %.2f g\r\n", cxg, cyg, czg);

	  HAL_Delay(200);
*/
	  //ADXL356 GPT ARCHITECTURE WORKING CODE END

	  //ADXL356 WORKING CODE START!!!
/*
	  HAL_ADC_Stop(&hadc1);                          // Ensure previous conversion is halted
	  HAL_ADC_Start(&hadc1);                         // Start ADC fresh
	  uint16_t x_raw = 0, y_raw = 0, z_raw = 0;
	      float x_voltage = 0, y_voltage = 0, z_voltage = 0;
	      float x_accel_g = 0, y_accel_g = 0, z_accel_g = 0;

	      // Read XOUT (Rank 1)
	      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	          x_raw = HAL_ADC_GetValue(&hadc1);

	      // Read YOUT (Rank 2)
	      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	          y_raw = HAL_ADC_GetValue(&hadc1);

	      // Read ZOUT (Rank 3)
	      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	          z_raw = HAL_ADC_GetValue(&hadc1);

	      // Convert ADC values to voltage
	      x_voltage = (x_raw / 4095.0f) * 3.3f;
	      y_voltage = (y_raw / 4095.0f) * 3.3f;
	      z_voltage = (z_raw / 4095.0f) * 3.3f;

	      // Convert voltage to acceleration (based on 10 g range)
	      x_accel_g = (x_voltage - 0.9f) / 0.08f;
	      y_accel_g = (y_voltage - 0.9f) / 0.08f;
	      z_accel_g = (z_voltage - 0.9f) / 0.08f;

	      // Print results
	      printf("X: %.3f V (%.2f g)\tY: %.3f V (%.2f g)\tZ: %.3f V (%.2f g)\r\n",
	              x_voltage, x_accel_g, y_voltage, y_accel_g, z_voltage, z_accel_g);
	  HAL_Delay(200);
*/

	  //ADXL356 WORKING CODE END!!!

	  //ADXL312 WORKING CODE START!!!
	  /*
	  int16_t x, y, z;
	  ADXL312_ReadXYZ(&x, &y, &z);
	  float xf = x / 85.0f;
	  float yf = y / 85.0f;
	  float zf = z / 85.0f;

	  printf("X: %.2f g, Y: %.2f g, Z: %.2f g\r\n", xf, yf, zf);

	  HAL_Delay(500);
*/
	  //ADXL312 WORKING CODE END!!!


	  //ADXL312 AXES TESTING CODE BEGIN
	  // After calibration…
/*
	  // Read a single sample and print g-values continuously
	      int16_t rx, ry, rz;
	      float cx, cy, cz;

	      ADXL312_ReadXYZ(&rx, &ry, &rz);
	      getCalibratedAccel(rx, ry, rz, &cx, &cy, &cz);

	      printf("X_g=%.3f, Y_g=%.3f, Z_g=%.3f\r\n", cx, cy, cz);
	      HAL_Delay(500);
*/
	  //ADXL312 AXES TESTING CODE END

//ADXL312 GPT Architecture Working Code Begin!!!
/*
	  int16_t rx, ry, rz;
	  float cx, cy, cz;

	  // 1) read raw counts
	  ADXL312_ReadXYZ(&rx, &ry, &rz);
	  // 2) apply calibration
	  getCalibratedAccel(rx, ry, rz, &cx, &cy, &cz);
	  // 3) print corrected values
	  printf("X: %.2f g, Y: %.2f g, Z: %.2f g\r\n", cx, cy, cz);

	  HAL_Delay(200);
*/
	  //ADXL312 GPT Architecture Working Code End!!!

	  //lis2mdl GPT ARCHITECTURE TESTING CODE BEGIN
/*
	  int16_t rx, ry, rz;
	  float mx, my, mz;

	  LIS2MDL_ReadRaw(&rx, &ry, &rz);
	  getCalibratedMag(rx, ry, rz, &mx, &my, &mz);
	  printf("Mag [µT] -> X: %.2f  Y: %.2f  Z: %.2f\r\n", mx, my, mz);

	  HAL_Delay(200);
*/
	    //LIS2MDL GPT ARCHITECTURE TESTING CODE END

	  //WORKING STEVALMKI181V1 CODE START!!!
/*
	  int16_t mag_x = 0, mag_y = 0, mag_z = 0;
	  uint8_t data[6] = {0};

	  // Read 6 bytes from OUTX_L_REG (0x68) to OUTZ_H_REG (0x6D)
	  HAL_I2C_Mem_Read(&hi2c1, 0x3C, 0x68, 1, data, 6, HAL_MAX_DELAY);

	  // Combine low/high bytes into signed 16-bit integers
	  mag_x = ((int16_t)(data[1] << 8 | data[0])) - offset_x;
	  mag_y = ((int16_t)(data[3] << 8 | data[2])) - offset_y;
	  mag_z = ((int16_t)(data[5] << 8 | data[4])) - offset_z;


	  // Convert raw magnetic data to microTesla (µT)
	  float mag_x_uT = mag_x * 0.15f;
	  float mag_y_uT = mag_y * 0.15f;
	  float mag_z_uT = mag_z * 0.15f;
	  // Print the data
	  // Print the converted values
	  printf("Magnetic Field [µT] -> X: %.2f\tY: %.2f\tZ: %.2f\r\n",
			  mag_x_uT, mag_y_uT, mag_z_uT);
	  HAL_Delay(200);  // Print every 200 ms
*/
	  //WORKING STEVALMKI181V1 CODE END!!!


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x0060112F;
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

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x006039F7;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

extern SPI_HandleTypeDef hspi1;

//ADXL356 TESTING CODE START

uint16_t Read_ADC_Channel(void) {
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

//ADXL356 TESTING CODE END

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
