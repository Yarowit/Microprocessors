/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l152c_discovery.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

///GPIOC pins used for motor communication (6,7,8,9)
#define M_L_CW 6 // 6 brown L1 PA1->PC6
#define M_L_CCW 7 // 7 orange L2 PA2->PC7
#define M_R_CW 8  // 8 green L3 PA3->PC8
#define M_R_CCW 9 // 9 blue L4 PA5->PC9

#define Buzzer 8 // PB8 ~ TIM4_CH3
#define SensorL 1 // PC1
#define SensorR 2 // PC2
#define BuzzerFreq 500 // half of the wave in milliseconds
#define SensorFreq 10 // checking period milliseconds

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
enum State {
    RS,LS,FS,SS,START_STATE
} state = FS, prevState = START_STATE;

unsigned char startFlag = 0;

int currentMaxSpeed = 20;

uint8_t command[1] = "-";
uint8_t prevcommand[1] = "-";

#define MAX_SPEED_MODIFIER 10
int speedModifier = MAX_SPEED_MODIFIER;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TS_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/// --------------------------- MOTOR FUNCTIONS ---------------------------
static inline void reset_CCRs(){
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
}

void backward(){
   reset_CCRs();
   /// PWM duty cycle
//   TIM3->CCR1 = 3/4*currentMaxSpeed;
//   TIM3->CCR4 = 3/4*currentMaxSpeed;
   TIM3->CCR1 = currentMaxSpeed;
   TIM3->CCR4 = currentMaxSpeed;

   uint8_t message[10] = "\nbackward\n";
   HAL_UART_Transmit(&huart1, message , 10, 10000);
}

void forward(){
  reset_CCRs();
  /// PWM duty cycle
//  TIM3->CCR2 = 3/4*currentMaxSpeed;
//  TIM3->CCR3 = 3/4*currentMaxSpeed;
  TIM3->CCR2 = currentMaxSpeed;
  TIM3->CCR3 = currentMaxSpeed;

//  TIM3->CCR2 = 13;
//  TIM3->CCR3 = 13;
  uint8_t message[9] = "\nforward\n";
  HAL_UART_Transmit(&huart1, message , 9, 10000);
}

void left(){
  reset_CCRs();
  /// PWM duty cycle
  TIM3->CCR3 = currentMaxSpeed;
//  TIM3->CCR3 = 15;
//  TIM3->CCR2 = currentMaxSpeed * 2/4; // other slower

  uint8_t message[6] = "\nleft\n";
  HAL_UART_Transmit(&huart1, message , 6, 10000);
}

void right(){
  reset_CCRs();
  /// PWM duty cycle
  TIM3->CCR2 = currentMaxSpeed;
//  TIM3->CCR2 = 15;
//  TIM3->CCR3 = currentMaxSpeed * 2/4; // other slower

  uint8_t message[7] = "\nright\n";
  HAL_UART_Transmit(&huart1, message , 7, 10000);
}

static inline void stop(){
  reset_CCRs();

  uint8_t message[6] = "\nstop\n";
  HAL_UART_Transmit(&huart1, message , 6, 10000);
}

/// --------------------------- IRQs ---------------------------


void TIM4_IRQHandler(void) {
  /// delay handler
  if((TIM4->SR & 0b0100) != 0){
//     TIM4->CCR3 += BuzzerFreq;
    if(startFlag == 0){
       startFlag = 1;
       TIM4->CCER |= 0b100000000;
       prevState = START_STATE;

       //start motors timer (TIM3)
       TIM3->CR1 |= 0x0001; // CEN = 1 -> Start counter
       TIM3->EGR |= 0x0001; // UG = 1 -> Generate update
       TIM3->SR = 0;
    }

     TIM4->SR &= ~0b0100 ;
  }
  /// buzzer handler
  if((TIM4->SR & 0b1000) != 0){
     TIM4->CCR3 += BuzzerFreq;
     TIM4->SR &= ~0b1000 ;
  }

  /// sensors handler
  // channel 1, so bit 2
  if((TIM4->SR & 0b0010) != 0){
    int L = GPIOC->IDR & (1<<SensorL);
    int R = GPIOC->IDR & (1<<SensorR);

    if(L && R){ // black line, stop
        state = SS;
    }else if(!L && !R){ // forward
        state = FS;
    }else if(!L && R){ // turn right
        state = RS;
    }else if(L && !R){ // turn left
        state = LS;
    }

    TIM4->CCR1 += SensorFreq; // update comparison time
    TIM4->SR &= ~0b0010 ; // Clear flag of CH1
  }
}


/// button handling
void EXTI0_IRQHandler(void){
  if (EXTI->PR!=0){
    TIM4->CR1 |= 0x0001; // CEN = 1 -> Start counter of TIM4
    EXTI->PR = 0x01;
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
  MX_ADC_Init();
  MX_TS_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /// ----------------------------- user button as EXTI ----------------------------

    GPIOA->MODER &= ~(0b11);
    EXTI->FTSR |= 0x01; // Enables falling edge in EXTI0
    EXTI->RTSR &= ~(0x01); // Disables rising edge in EXTI0
    SYSCFG->EXTICR[0] = 0; // EXTI0 linked to GPIOA (i.e. USER button = PA0)
    EXTI->IMR |= 0x01; // Enables EXTI0
    NVIC->ISER[0] |= (1 << 6);  // Enables IRQ for EXTI0 (6th position)
                                // If USER button is pressed, a falling edge is found,
                                // the flag is activated, the IRQ is launched to the NVIC,
                                // the NVIC activates the external IRQ signal.
                                // The CPU calls the EXTI0 ISR

  /// initialize motor pins as AFR
    GPIOC->MODER &= ~(   0b11 * (1<<(2*M_L_CW))
                      |  0b11 * (1<<(2*M_L_CCW))
                      |  0b11 * (1<<(2*M_R_CW))
                      |  0b11 * (1<<(2*M_R_CCW)) );

    GPIOC->MODER |= (    0b10 * (1<<(2*M_L_CW))
                      |  0b10 * (1<<(2*M_L_CCW))
                      |  0b10 * (1<<(2*M_R_CW))
                      |  0b10 * (1<<(2*M_R_CCW)) );

    /// associate with TIM3
//    GPIOC->AFR[0] = 0;
//    GPIOC->AFR[1] = 0;
    GPIOC->AFR[0] &=  ~( 0b1111 << (4*M_L_CW)
                       | 0b1111 << (4*M_L_CCW));
    GPIOC->AFR[1] &=  ~( 0b1111 << (4*(M_R_CW - 8))
                       | 0b1111 << (4*(M_R_CCW - 8)));

    GPIOC->AFR[0] |=       0b10 << (4*M_L_CW)
                         | 0b10 << (4*M_L_CCW);
   GPIOC->AFR[1] |=       0b10 << (4*(M_R_CW - 8))
                         | 0b10 << (4*(M_R_CCW - 8));

  /// buzzer
  // as output
  GPIOB->MODER &= ~( 0b11 * (1<<(2*Buzzer)));
  GPIOB->MODER |= 0b10 * (1<<(2*Buzzer));
  // AFR
  GPIOB->AFR[1] &= ~(0b1111);
  GPIOB->AFR[1] |= 0b0010;
  // buzzer timer
  TIM4->CCR3 = BuzzerFreq; // channel 3
  TIM4->CCMR2 &= ~(0x00FF);
  TIM4->CCMR2 |= 0b0110000; // also TOC

  // set buzzer to forward mode
  TIM4->CCMR2 &= ~(0x00FF);
  TIM4->CCMR2 |= 0b0010000;

  /// ------------------------------ ADC setup --------------------------------------
    /// PA5

    GPIOA->MODER |= 0b11<<(2*5); // PA5 as analog
    ADC1->CR2 &= ~(0x00000001); // ADON = 0 (ADC powered off)
    ADC1->CR1 = 0x00000000; // OVRIE = 0 (overrun IRQ disabled)
                            // RES = 00 (resolution = 12 bits)
                            // SCAN = 0 (scan mode disabled)
                            // EOCIE = 0 (EOC IRQ disabled)
    ADC1->CR2 = 0b10000010010;  // EOCS = 1 (EOC is activated after each conv.)
                                // DELS = 001 (delay till data is read)
                                // CONT = 1 (continuous conversion)
    ADC1->SQR1 = 0x00000000; // 1 channel in the sequence
    ADC1->SQR5 = 0x00000005; // The selected channel is AIN5
    ADC1->CR2 |= 0x00000001; // ADON = 1 (ADC powered on)
    while ((ADC1->SR&0x0040)==0); // If ADCONS = 0, I wait till converter is ready
    ADC1->CR2 |= 0x40000000; // When ADCONS = 1, I start conv. (SWSTART = 1)


  /// initialize timer TIM4
  // CH3 - buzzer, CH1 - IR measurement
  TIM4->CR1 = 0x0000; // ARPE = 0 -> No PWM, it is TOC
  // CEN = 0; Counter OFF
  TIM4->CR2 = 0x0000; // Always "0" in this course
  TIM4->SMCR = 0x0000; // Always "0" in this course

  TIM4->PSC = 32000 - 1; // prescaler makes measurements every 1ms
  TIM4->CNT = 0; // Initialize the counter to 0
  TIM4->ARR = 0xFFFF; // Recommended value = FFFF
  TIM4->DIER = 0b10; // interrupt enable CCyIE = 1

  ///---------------------- initialize TIM3 ----------------------
    /// PWM
    TIM3->CR1 = 0x0080; // ARPE = 1 -> Is PWM; CEN = 0; Counter OFF
    TIM3->CR2 = 0x0000; // Always 0 in this course
    TIM3->SMCR = 0x0000; // Always 0 in this course

//    TIM3->PSC = 320-1;
//    TIM3->PSC = 32000-1;
    TIM3->PSC = 48000-1;
    TIM3->CNT = 0; // Initialize counter to 0
    TIM3->ARR = 20; // PWM Frequency to 100 Hz and only 10 steps
    reset_CCRs(); // motors stopped at start
    //TIM3->CCR2 = 10;
    //TIM3->CCR1 = 10;
//    TIM3->CCR2 = 20;
    TIM3->DIER = 0x0000;
//    TIM3->CCMR1 = 0x6868; // CCyS = 0 (TOC, PWM)
                          // OCyM = 110 (PWM starting in 1)
                          // OCyPE = 1 (with preload)
    TIM3->CCMR1 = 0x6868;
    TIM3->CCMR2 = 0x6868;
    TIM3->CCER = 0x1111;  // CCyP = 0 (always in PWM)
                          // CCyE = 1 (hardware output activated)

   TIM3->CR1 |= 0x0001; // CEN = 1 -> Start counter
   TIM3->EGR |= 0x0001; // UG = 1 -> Generate update
   TIM3->SR = 0;


  /// sensors

  // as input (00)
  GPIOC->MODER &= ~( 0b11 * (1<<(2*SensorL)));
  GPIOC->MODER &= ~( 0b11 * (1<<(2*SensorR)));
  // sensors timer
  TIM4->CCR1 = SensorFreq; // channel 1
  TIM4->CCMR1 = 0x0000; // also TOC
  // delay timer
  TIM4->CCR2 = 2000; // 2s

  TIM4->CCER &= ~(0x0380); // CCyP = 0 (always in TOC)
//  TIM4->CCER |= 0b100000000; <- in EXTI

  // Counter enabling
  /// DONT start the counter
  TIM4->CR1 &= ~(1);
//  TIM4->CR1 |= 0x0001; // CEN = 1 -> Start counter
  TIM4->EGR |= 0x0001; // UG = 1 -> Generate an update event to update all registers
  TIM4->SR = 0; // Clear counter flags

  /// Enabling TIM4_IRQ at NVIC (position 30).
  NVIC->ISER[0] |= (1 << 30);


  HAL_UART_Receive_IT(&huart1, command , 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(command[0] != prevcommand[0]){
      if(command[0] != 'A'){
        // turn off the buzzer
        TIM4->CCMR2 &= ~(0x00FF);
        TIM4->CCMR2 |= 0b0010000; // pin to 1
      }
      prevcommand[0] = command[0];
      switch(command[0]){
        case 'F':
          forward();
          break;
        case 'S':
          stop();
          break;
        case 'B':
          backward();
          break;
        case 'L':
          left();
          break;
        case 'R':
          right();
          break;
        default:
          break;
      }
      // change of speed
      if('0' <= command[0] && command[0] <= '9'){
        speedModifier = command[0] - '0' + 1;
        currentMaxSpeed = 10 + (ADC1->DR * 10 / (1<<12) * speedModifier/MAX_SPEED_MODIFIER);


        uint8_t message[12] = "\nNew speed: ";
        HAL_UART_Transmit(&huart1, message , 12, 10000);
//        command[0]++;
//        uint8_t message5[1] = command + 1;
        HAL_UART_Transmit(&huart1, command , 1, 10000);
        uint8_t message2[4] = "/9\n";
        HAL_UART_Transmit(&huart1, message2 , 4, 10000);

      }


      // autonomous
      if(command[0] == 'A'){
        uint8_t message[12] = "\nAutonomous\n";
        HAL_UART_Transmit(&huart1, message , 12, 10000);
        int breakFlag = 1;
        while(breakFlag)
        if((state != prevState || currentMaxSpeed != 10 + (ADC1->DR * 10 / (1<<12)* speedModifier/MAX_SPEED_MODIFIER)) && startFlag){/// !!!!!!
        //      if((state != prevState ) && startFlag){/// !!!!!!
                  prevState = state;
                  currentMaxSpeed = 10 + (ADC1->DR * 10 / (1<<12) * speedModifier/MAX_SPEED_MODIFIER);
        //          currentMaxSpeed = 12;
                  switch(state){
                    case FS:
                        forward();
                        TIM3->EGR |= 0x0001; // UG = 1 -> Generate update
                        TIM3->SR = 0;
                        // buzzer
                        TIM4->CCMR2 &= ~(0x00FF);
                        TIM4->CCMR2 |= 0b0010000; // pin to 1
                        break;
                    case LS:
                        left();
                        TIM3->EGR |= 0x0001; // UG = 1 -> Generate update
                        TIM3->SR = 0;
                        // buzzer
                        TIM4->CCMR2 &= ~(0x00FF);
                        TIM4->CCMR2 |= 0b0100000; // pin to 0
                        break;
                    case RS:
                        right();
                        TIM3->EGR |= 0x0001; // UG = 1 -> Generate update
                        TIM3->SR = 0;
                        // buzzer
                        TIM4->CCMR2 &= ~(0x00FF);
                        TIM4->CCMR2 |= 0b0100000; // pin to 0
                        break;
                    case SS:
                        stop();
                        TIM3->EGR |= 0x0001; // UG = 1 -> Generate update
                        TIM3->SR = 0;
                        // buzzer
                        TIM4->CCMR2 &= ~(0x00FF);
                        TIM4->CCMR2 |= 0b0110000; // toggle
        //                EXTI->PR = 1; /// Reset EXTI flag
                        breakFlag = 0;
                        break;
                    default:
                      break;
                }
            }
      }

    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : SEG14_Pin SEG15_Pin SEG16_Pin SEG17_Pin
                           SEG18_Pin SEG19_Pin SEG20_Pin SEG21_Pin
                           SEG22_Pin SEG23_Pin */
  GPIO_InitStruct.Pin = SEG14_Pin|SEG15_Pin|SEG16_Pin|SEG17_Pin
                          |SEG18_Pin|SEG19_Pin|SEG20_Pin|SEG21_Pin
                          |SEG22_Pin|SEG23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin COM0_Pin
                           COM1_Pin COM2_Pin SEG12_Pin */
  GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG2_Pin|COM0_Pin
                          |COM1_Pin|COM2_Pin|SEG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG6_Pin SEG7_Pin SEG8_Pin SEG9_Pin
                           SEG10_Pin SEG11_Pin SEG3_Pin SEG4_Pin
                           SEG5_Pin SEG13_Pin COM3_Pin */
  GPIO_InitStruct.Pin = SEG6_Pin|SEG7_Pin|SEG8_Pin|SEG9_Pin
                          |SEG10_Pin|SEG11_Pin|SEG3_Pin|SEG4_Pin
                          |SEG5_Pin|SEG13_Pin|COM3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  HAL_UART_Receive_IT(huart , command, 1); // Vuelve a activar Rx por haber acabado el buffer
}
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
