/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include "math.h"
#include "HMC5883L.h"
#include "lcd_i2c.h"

//#include "i2c.h"

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
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile const float c=5.2*3.14; // Licznik impulsow
volatile const float k=(5.2*3.14)/3840;
volatile uint16_t AD_RES = 0, Vamb, DC_Multiplier,al;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

 {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

 }*/
char data[100];
char string[100];
int liczba;
int dystans = 0;
int kierunek = 0;
int flagaA = 0;
int flagaB = 0;
volatile int licznik=0;


uint8_t Data[6];
int16_t rawData[3];
float scaledData[3];
volatile float azymuth=0;
char msg[30];
int16_t xoffset=0, yoffset=0;
float direction = 359.5;

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;
uint8_t Distance = 0;

struct lcd_disp disp;
//Zatrzymanie silników
void stop()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
}

//Ruch prosto
void move_forward(int time)
{

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);

    HAL_Delay(time);
    stop();
}

//Obrót w prawo przez określony czas
void turn_right_time(int time)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
    HAL_Delay(time);
    stop();
}

//Obrót w prawo (ciągły)
void turn_right()
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
}

//Obrót w lewo przez określony czas
void turn_left_time(int time)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,SET);
    HAL_Delay(time);
    stop();
}

//Obrót w lewo (ciągły)
void turn_left()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,SET);

}

//Wyślij napis przez UART
void send_string(char* s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 100);
}

void send_char(char s)
{
    HAL_UART_Transmit(&huart2,(uint8_t*)&s, sizeof(s), 100);
}

void magnetometer_init()
{
    uint8_t adres = 0x80;

    HAL_I2C_Mem_Write(&hi2c1, 0x0D<<1, 0x0A, 1, &adres, 1, HAL_MAX_DELAY);
    adres = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, 0x0D<<1, 0x0B, 1, &adres, 1, HAL_MAX_DELAY);
    uint8_t rejestr = 0b00100001;
    		//0x1D;
    HAL_I2C_Mem_Write(&hi2c1, 0x0D<<1, 0x09, 1, &rejestr, 1, HAL_MAX_DELAY);
}


//Pojedynczy pomiar
void magnetometer_measure()
{
    //magnetometer_init();
    uint8_t adres = 0x80;
   // HAL_I2C_Mem_Write(&hi2c1, 0x0D<<1, 0x0A, 1, &adres, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, 0x0D<<1, 0x00, 1, (uint8_t*)Data, 6, 100);
    for(int i=0; i<3; i++)
    {
        rawData[i]=((int16_t)((uint16_t)Data[2*i+1] << 8) + Data[2*i]);
    }

    scaledData[0]= ((float)rawData[0]- xoffset)/3000.0;
    scaledData[1]= ((float)rawData[1]- yoffset)/3000.0;

    azymuth = atan2((float)scaledData[1],(float)scaledData[0]);

    if (azymuth < 0)
        azymuth += 2 * M_PI;
    if (azymuth > 2*M_PI)
        azymuth -= 2 * M_PI;

    azymuth = azymuth * 180/ M_PI;

    if (azymuth >= 0 && azymuth <= 270)
        azymuth = azymuth + 90.00;
    else if (azymuth>270)
        azymuth = azymuth - 270;

    sprintf(msg, "Azymut: %f \r\n", (float)azymuth);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);

}


//Kalibracja magnetometru
void calibrate()
{

    float xmin,ymin,xmax,ymax;
    sprintf(msg, "Start \r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);

   // magnetometer_measure();
    xmin=xmax=rawData[0];
    ymin=ymax=rawData[1];
    turn_left();

    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET)

    {
		HAL_I2C_Init(&hi2c1);

        // magnetometer_measure();
 		//HAL_I2C_DeInit(&hi2c1);

        if(rawData[0] < xmin)
            xmin = rawData[0];
        if(rawData[0] > xmax)
            xmax = rawData[0];
        if(rawData[1] < ymin)
            ymin = rawData[1];
        if(rawData[1] > ymax)
            ymax = rawData[1];
    }

    sprintf(msg, "xmin: %f \r\n", (float)xmin);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);
    sprintf(msg, "xmax: %f \r\n", (float)xmax);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg), 100);

    xoffset = (xmin+xmax)/2;
    yoffset = (ymin+ymax)/2;
    stop();

}

//Obrót w kierunku polnocy
void north()
{
   // magnetometer_measure();
    if((azymuth >= 0.00) && (azymuth <= 180.00))
        turn_left();
    else
        turn_right();

    while(azymuth >= 1.00 && azymuth <=359.00)
    {
  //      magnetometer_measure();
    }
    stop();
}

void magnetic_direction()
{
	float heading = kierunek;
	   // magnetometer_measure();
	    if (azymuth >= 0.00 && azymuth <=180.00 && heading >=0.00 && heading <= 180.00)
	    {
	        if((heading - azymuth) <= 0.00)
	            turn_left();
	        else
	            turn_right();
	    }
	    else if (azymuth >= 0.00 && azymuth <=180.00 && heading >180.00 && heading <= 360.00)
	    {
	        {
	            if((heading - azymuth) >= 180.00)
	                turn_left();
	            else
	                turn_right();
	        }
	    }
	    else if (azymuth > 180.00 && azymuth <=360.00 && heading >=0.00 && heading <= 180.00)
	    {
	        if((heading - azymuth) >= -180.00)
	            turn_left();
	        else
	            turn_right();
	    }

	    else if (azymuth > 180.00 && azymuth <=360.00 && heading >180.00 && heading <= 360.00)
	    {
	        if((heading - azymuth) <= 0.00)
	            turn_left();
	        else
	            turn_right();
	    }

	    if(heading >= 1.00 && heading <= 359.00)
	    {
	        while(1)
	        {
	            if((azymuth <= heading + 1.00) && (azymuth >= heading - 1.00))
	                break;
	           // magnetometer_measure();
	        }
	    }
	    else if(heading >= 0.00 && heading < 1.00)
	    {
	        while(1)
	        {
	            if((azymuth <= heading + 1.00 ) || (azymuth >= heading - 1.00 + 360.00))
	                break;
	           // magnetometer_measure();
	        }
	    }
	    else if(heading > 359.00 && heading <= 360.00)
	    {
	        while(1)
	        {
	            if((azymuth <= heading + 1.00 -360.00 ) || (azymuth >= heading - 1.00))
	                break;
	          //  magnetometer_measure();
	        }
	    }


	    stop();

}

//Zmiana predkosci silników
void change_speed(uint32_t pulse)
{
    htim1.Instance->CCR1=pulse;
    htim1.Instance->CCR2=pulse;
}




void move_forward_1()
{

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);

   // HAL_Delay(time);
  //  stop();
}


//Światła
void lights(){

	 // Start ADC Conversion
	    	        HAL_ADC_Start(&hadc1);
	    	       // Poll ADC1 Perihperal & TimeOut = 1mSec
	    	        HAL_ADC_PollForConversion(&hadc1, 1);
	    	       // Read The ADC Conversion Result & Map It To PWM DutyCycle
	    	        AD_RES = HAL_ADC_GetValue(&hadc1);

	if(AD_RES<310)
	   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);



	if(AD_RES>310)
	   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);


	}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10){ // Jeżeli przerwanie pochodzi od timera 10
		lights();
	}
	if(htim->Instance == htim5.Instance)
	{
	    HAL_I2C_Mem_Read(&hi2c1, 0x0D<<1, 0x00, 1, (uint8_t*)Data, 6, 100);
	    for(int i=0; i<3; i++)
		    	    {
		    	        rawData[i]=((int16_t)((uint16_t)Data[2*i+1] << 8) + Data[2*i]);
		    	    }

		    	    scaledData[0]= ((float)rawData[0]- xoffset)/3000.0;
		    	    scaledData[1]= ((float)rawData[1]- yoffset)/3000.0;

		    	    azymuth = atan2((float)scaledData[1],(float)scaledData[0]);

		    	    if (azymuth < 0)
		    	        azymuth += 2 * M_PI;
		    	    if (azymuth > 2*M_PI)
		    	        azymuth -= 2 * M_PI;

		    	    azymuth = azymuth * 180/ M_PI;

		    	    if (azymuth >= 0 && azymuth <= 270)
		    	        azymuth = azymuth + 90.00;
		    	    else if (azymuth>270)
		    	        azymuth = azymuth - 270;

		    	  //  if(licznik%50000==0){
		    	    	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		    	    //	   }

		    	    	//    licznik++;
		    	    	//   if(licznik>=50001)
		    	    	//  licznik=0;
	}

}

// Funkcja delay od timera 4
void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim4,0);
	while (__HAL_TIM_GET_COUNTER(&htim4)<time);
}

//Funkcja do czujnika odległości
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == htim4.Instance)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) //if the interrupt source is channel1
		{
			if(Is_First_Captured == 0) // if the first value is not captured
			{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); //read the first value
			Is_First_Captured = 1; // set the first captured as true
			//Change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			else if (Is_First_Captured == 1) // is the first already captured
			{
				IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); //read the second value
				__HAL_TIM_SET_COUNTER(htim,0); // reset the counter

				if(IC_Val2 > IC_Val1)
				{
					Difference = IC_Val2 - IC_Val1;
				}
				else if(IC_Val1 > IC_Val2)
				{
					Difference = IC_Val1 - IC_Val2;
				}
				Distance = Difference *.034/2;

				Is_First_Captured = 0; // set it back to false

				//set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC3);
				}
		}

	}
}


//Funkcja do czujnika odległości
void HCSR04_Read(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // pull the TRIG pin HIGH
	delay(10); // wait for 10 us
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // pull the TRIG pin LOW

	__HAL_TIM_ENABLE_IT(&htim4,TIM_IT_CC3);
}

//Ruch na zadaną odległość
void move_(float x){
	TIM2->CNT=0;
	  move_forward_1();
	  uint8_t is_collison = 0;
	if(Distance <= 300)
	{
		is_collison = 1;
	}
	while(TIM2->CNT*k<=x && is_collison == 0){
		if(x>c && TIM2->CNT==3839){
			x=x-c;
			HCSR04_Read();
			HAL_Delay(1);
			}

	}

	if(TIM2->CNT*k>=x || is_collison == 1){
		al=TIM2->CNT;
//		stop();
		obstacle_bypass();
	//HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	}
}

// Omijanie przeszkody
void obstacle_bypass()
{
	float a_azymuth = azymuth+90;
	kierunek = a_azymuth;
	magnetic_direction();
	move_(50);
	kierunek = azymuth - 90;
	magnetic_direction();

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
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
   __HAL_RCC_I2C1_CLK_ENABLE();
   __I2C1_CLK_ENABLE();
   __HAL_RCC_USART2_CLK_ENABLE();
   HAL_TIM_Base_Start_IT(&htim1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
   __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
   __HAL_RCC_GPIOA_CLK_ENABLE();
   	__HAL_RCC_USART2_CLK_ENABLE();
   	__HAL_RCC_ADC1_CLK_ENABLE();
    HAL_TIM_Base_Start_IT(&htim5);

   //	HAL_ADCEx_Calibration_Start(&hadc1);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    Vamb = HAL_ADC_GetValue(&hadc1);
    DC_Multiplier = 65535/(4096-Vamb);



    //konfiguracja wyswietlacza
    disp.addr = (0x27 << 1);
    disp.bl = true;
    lcd_init(&disp);
    sprintf((char *)disp.f_line, "Konfiguracja");   // 1 linia wyświetlacza
    sprintf((char *)disp.s_line, "wyswietlacza."); // 2 inia wyświetlacza
    lcd_display(&disp);

    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
    HCSR04_Read();
    HAL_Delay(1000);


    magnetometer_init();
    calibrate();
    /*
    move_(10);
    HAL_Delay(1000);
    move_(20);
    HAL_Delay(1000);
    move_(50);
    HAL_Delay(1000);
    move_(100);
    HAL_Delay(1000);
    */

//
//
//     move_(88);


//    while(1){
//
//
//    	}
    /*	if(k*cnt>10){
    	stop();
    	}
    	}*/

    /*HAL_Delay(1000);
    turn_left_time(1000);
    north();
    HAL_Delay(1000);
    turn_right_time(1000);
    north();*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
    	  turn_left_time(5000);
    	    turn_right_time(5000);
    	    move_(100);

    	/*   if(licznik%50000==0){
    		    	    sprintf((char *)disp.s_line, "azymut: %d", (int)azymuth); // 2 inia wyświetlacza
    		    	      lcd_display(&disp);
    		    	    	}

    		    	    licznik++;
    		    	    if(licznik>=50001)
    		    	    	licznik=0;

    		    	    turn_left();
*/



      HCSR04_Read();

      if(flagaA==1)
      {
    	  move_((float)dystans);
    	  flagaA=0;
      }
      else if(flagaB==1)
      {
    	  direction=kierunek;
    	  magnetic_direction();
    	  flagaB=0;
    	  HAL_Delay(500);
      }


     /* HAL_Delay(200);

      move_(100);
		  HAL_Delay(200);
////		magnetic_direction();
		  turn_left_time(4000);

		  HAL_Delay(200);
		  turn_right_time(4000);

		  HAL_Delay(200);

//    	lcd_send_string("Hello World!");
//    	HAL_Delay(200);
//    	lcd_clear();
//    	HAL_Delay(250);
   //     HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    //    HAL_Delay(500);
   //     magnetometer_measure();*/
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3839;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65534;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 7999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 80;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
