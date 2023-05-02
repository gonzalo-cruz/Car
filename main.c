/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "myfile.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIME_FULL_STOP 50000
#define TIME_BUZZER_TOC 30000
#define TIME_TRIGGER_TOC 320
#define LIMITE 20
#define PWM_PREESCALER 319
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned char t_value = 'S';
unsigned int gearbox[5];//THE SPEEDS IN THIS PROGRAMS WILL FUNCTION LIKE A GEARBOX,0 IS LOWEST, 4 IS HIGHEST
unsigned char is_buzzer_on = 0;
unsigned char blinking = 0;
unsigned char nturns = 0;
unsigned char turning_time = 0;
unsigned char stopped = 0;
unsigned short pot_value = 0;
unsigned short tiempo = 0;
unsigned int distance  = 0;

unsigned char buff_rx[8];
unsigned char buff_tx[256];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_String(char *c){
  int i = 0;
  while (*c != 0){//SENDS CHARACTERS UNTIL THE STRIGN FINISHES
    buff_tx[i] = *c;
    c++;
    i++;
  }
  buff_tx[i] = '\0';
  HAL_UART_Transmit_IT(&huart1, buff_tx, (i)*sizeof(unsigned char));
}

void Start_TIM2(void){
  TIM2 -> CR1 |= 0x0001;
  TIM2 -> EGR |= 0x0001;
  TIM2 -> SR = 0x0000;
}

void Stop_TIM2(void){
  TIM2 -> CR1 &= 0x0000;
  TIM2 -> EGR |= 0x0001;
  TIM2 -> SR = 0x0000;
}

void Start_TIM3(void){
  TIM3 -> CR1 |= 0x0001;
  TIM3 -> EGR |= 0x0001;
  TIM3 -> SR = 0;
}

void Stop_TIM3(void){
  TIM3 -> CR1 &= 0x0000;
  TIM3 -> EGR |= 0x0001;
  TIM3 -> SR = 0x0000;
}
void Full_Stop(void){//IN THIS FUNCTION I SET ALL PINS TO 0 AND THE PWM TO 0 ALSO
  stopped=0;
  TIM4 -> CCR3 = gearbox[0];//THIS MAKES ALL THE WHEELS STOP
  TIM4 -> CCR4 = gearbox[0];
  GPIOA -> BSRR = (1<<(11+16));
  GPIOA -> BSRR = (1<<(12+16));
}

void Full_Forward(void){
  TIM4 -> CCR3 = gearbox[4];
  TIM4 -> CCR4 = gearbox[4];
  GPIOA -> BSRR = (1<<(11+16));
  GPIOA -> BSRR = (1<<(12+16));
}

void Full_Backward(void){
  TIM4 -> CCR3 = gearbox[0];
  TIM4 -> CCR4 = gearbox[0];
  GPIOA -> BSRR = (1<<(11));
  GPIOA -> BSRR = (1<<(12));
}

void R_Turn(void){
  //Left wheel stopped, right wheel backwards
  turning_time = 0;

  TIM4 -> CCR4 = gearbox[0];
  GPIOA -> BSRR = (1<<(12+16));

  TIM4 -> CCR3 = gearbox[0];
  GPIOA -> BSRR = (1<<11);
  while(turning_time!=3);
  Full_Stop();
}

void L_Turn(void){
  turning_time = 0;
  TIM4 -> CCR3 = gearbox[0];
  GPIOA -> BSRR = (1<<(11+16));
  TIM4 -> CCR4 = gearbox[0];
  GPIOA -> BSRR = (1<<12);
  while(turning_time!=3);
  Full_Stop();
}

void Velocity_Selector(void){
  while ((ADC1 -> SR & 0x0040)==0);//wait until the adc is ready
  ADC1->CR2 |= 0x40000000; //START CONVERSION
  while ((ADC1->SR&0x0002)==0); // WAIT UNTIL END OF CONVERSION
  pot_value = ADC1 -> DR; //STORE THE ADC VALUE, THEN SELECT THE 4 VELOCITIES (DUTY CYCLES)
  if((pot_value >=0) && (pot_value <= 1023)){//SLOWER CASE
    gearbox[0] = 0;//The gear 0 will always be full stop
    gearbox[1] = 100;
    gearbox[2] = 120;
    gearbox[3] = 140;
    gearbox[4] = 160;//This is the maximum speed in the slower case
  } else if ((pot_value > 1023) && (pot_value <= 2047)){//MEDIUM LOW CASE
    gearbox[0] = 0;//The gear 0 will always be full stop
    gearbox[1] = 180;
    gearbox[2] = 200;
    gearbox[3] = 220;
    gearbox[4] = 240;
  } else if ((pot_value > 2047) && (pot_value <= 3071)){// MEDIUM HIGH CASE
    gearbox[0] = 0;//The gear 0 will always be full stop
    gearbox[1] = 180;
    gearbox[2] = 220;
    gearbox[3] = 260;
    gearbox[4] = 270;
  } else {//FULL SPEED AVAILABLE
    gearbox[0] = 0;//The gear 0 will always be full stop
    gearbox[1] = 220;
    gearbox[2] = 240;
    gearbox[3] = 280;
    gearbox[4] = 320;
  }
}

void Geardown(void){//MAYBE WE WILL NEED TO ADD MORE SPEEDS BUT THE IDEA IS THE SAME
  if((distance < 30000)&&(distance > 23333)){
    TIM4 -> CCR3 = gearbox[3];
    TIM4 -> CCR4 = gearbox[3];
  } else if((distance < 23333)&&(distance > 16666)){
    TIM4 -> CCR3 = gearbox[2];
    TIM4 -> CCR4 = gearbox[2];
  } else if((distance < 16666)&&(distance > 10000)){
    TIM4 -> CCR3 = gearbox[1];
    TIM4 -> CCR4 = gearbox[1];
  }
}

void OAP(void){//OAP STANDS FOR OBJECT AVOIDANCE PROTOCOL
  stopped = 0;
  Full_Stop(); //tiene que esperar medio segundo
  //send_String("CAR STOPPED\r\n");
  while(stopped !=3);//Wait until the timer finishes

  if(nturns==0){
  R_Turn();
  send_String("TURNING RIGHT\r\n");
  nturns++;
  } else if(nturns==1){
    L_Turn();
    send_String("TURNING LEFT\r\n");
    stopped=0;
    Full_Stop(); //tiene que esperar medio segundo
    while(stopped !=1);//Wait until the timer finishes
    L_Turn();
    send_String("TURNING LEFT\r\n");
    nturns++;
  } else{
    L_Turn();
    send_String("TURNING LEFT\r\n");
    nturns = 0;
  }
}



void On_Off_Blink(void){
  if(distance < 10000){ //buzzer is fully active
    GPIOA -> BSRR = (1<<17);
    blinking = 0;
    is_buzzer_on = 1;
  } else if((distance < 30000)&&(distance > 10000)){
    blinking = 1;
    //Geardown();
  } else{
     GPIOA -> BSRR = (1<<1);
     blinking = 0;
     is_buzzer_on = 0;
     //Full_Forward();
  }
}
void Blinking(void){
  if(blinking == 1){
    if(is_buzzer_on == 0){
      GPIOA -> BSRR = (1<<17);
      is_buzzer_on = 1;
    } else{
      GPIOA -> BSRR = (1<<1);
      is_buzzer_on = 0;
    }
  }
}


void TIM2_IRQHandler(void){
  if((TIM2 -> SR & 0x0002) == 0){
    //STORE THE TIME
    tiempo = TIM2->CCR1;
    tiempo -= 250;
    if (tiempo<0) tiempo += 0xFFFF;
    distance = calcular_distancia(tiempo);
    Start_TIM3();
    GPIOD -> BSRR = (1<<2);
    //STOP TIMER 2
    TIM2 -> CR1 &= 0x0000;
    TIM2 -> EGR |= 0x0001;
    On_Off_Blink();
  }
  if((TIM2 -> SR & 0x0004) != 0){
    Blinking();
  }
  if((TIM2 -> SR & 0x0008) != 0){
    turning_time++;
    stopped++;

  }
  TIM2 -> SR = 0x0000;
}

void TIM3_IRQHandler(void){
  // WHEN THE COMPARISON IS SUCCESSFUL
  if((TIM3 -> SR & 0x0002) != 0){
    //DEACTIVATE TRIGGER
    GPIOD -> BSRR = (1<<(2+16));
    //STARTS TIMER 2 TIC
    //STARTS ECHO RECEPTION (DONE AUTOMATICALLY)
    Start_TIM2();
    //CLEARS ITSELF
    Stop_TIM3();
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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //LAB 3 INITIALIZATION (Potentiometer, ADC, PWM, maybe extra timers)

  // PA4 (POTENTIOMETER) INITIALIZATION
  GPIOA -> MODER |= (1<<4);//ANALOG FUNCTIONALITY
  GPIOA -> MODER |= (1<<5);

  ADC1 -> CR2 &= ~(1<<1);//MAKE SURE THE POWER IS OFF
  ADC1 -> CR1 = 0x00000000;
  ADC1 -> CR2 |= 0x00000400;//EOCS IS ACTIVATED AT THE END OF EACH REGULAR CONVERSION
  ADC1 -> SQR1 = 0x00000000;//I JUST WANT ONE CONVERSION
  ADC1 -> SQR5 = 0x00000004;
  ADC1 -> CR2 |= 0x00000001;//POWER ON
  //PWM4 INITIALIZATION
  TIM4 -> CR1 = 0x0080; //ACTIVATE PWM MODE
  TIM4 -> CR2 = 0x0000;
  TIM4 -> SMCR = 0x0000;
  TIM4 -> PSC = PWM_PREESCALER;
  TIM4 -> CNT = 0;
  TIM4 -> ARR = 320;//CAMBIAR A 320 PETICION DEL TEACHER
  TIM4 -> CCR3 = 320;//AT FIRST WE WILL SELECT THE FULL VELOCITY
  TIM4 -> CCR4 = 320;//AFTERWARDS WE WILL CHANGE IT USING THE ADC
  TIM4 -> DIER = 0x0000;//no interrupts

  TIM4 -> CCMR2 = 0x6868;//IN BOTH CHANNELS GOES THE SAME CONFIGURATION
  TIM4 -> CCER = 0x1100;// ALWAYS IN PWM AND HARDWARE OUTPUT ACTIVE
  TIM4 -> CR1 |= 0x0001;
  TIM4 -> EGR |= 0x0001;
  TIM4 -> SR = 0;
  //LAB 1 INITIALIZATION

  //PA6 is going to enable the first motor in the driver
  GPIOA -> MODER |= (1<<12);
  GPIOA -> MODER &= ~(1<<13);
  //First motor enable pin
  GPIOA -> BSRR = (1<<6);


  //PB8 -> IN1, GOES ASSOCIATED  TO AF 2
  GPIOB -> MODER |= (1<<17);
  GPIOB -> MODER &= ~(1<<16);
  GPIOB -> AFR[1] |= 0x0002;//ASSOCCIATED TO TIM 4
  //PA11 -> IN2
  GPIOA -> MODER |= (1<<22);
  GPIOA -> MODER &= ~(1<<23);
  //PB9 -> IN3 ASSSOCIATED WITH AF2
  GPIOB -> MODER |= (1<<19);
  GPIOB -> MODER &= ~(1<<18);
  GPIOB -> AFR[1] |= 0x0020; //GOES TO TIM 4 THOSE WILL USE PWM
  //PA12 -> IN4
  GPIOA -> MODER |= (1<<24);
  GPIOA -> MODER &= ~(1<<25);


  //LAB 2 INITIALIZATION
  //PD2, IS AN OUTPUT
  GPIOD -> MODER &= ~(1<<5);
  GPIOD -> MODER |= (1<<4);
  //PA5 IS A TIC IS ECHO
  GPIOA -> MODER &= ~(1<<10);
  GPIOA -> MODER |= (1<<11);
  GPIOA -> AFR[0] &= ~(1 << 21);
  GPIOA -> AFR[0] &= ~(1 << 22);
  GPIOA -> AFR[0] &= ~(1 << 23);
  GPIOA -> AFR[0] |= (1 << 20);

  //PA1 USES A TOC USED TIMER 3 BUT IS AN OUTPUT, THIS IS THE BUZZER
  GPIOA -> MODER &= ~(1<<3);
  GPIOA -> MODER |= (1<<2); //MODER 10 TO MAKE PA1 OUTPUT

  GPIOA -> BSRR |= (1<<1); //Silence the buzz

  //Now it is time to set up the timers TIM2 and TIM3

  //FIRST TIM2 TIC OF ECHO

  //I JUST DO THE ORDINARY THINGS TO INITIALIZE
  TIM2 -> CR1 = 0x0000;
  TIM2 -> CR2 = 0x0000; //ALWAYS 0
  TIM2 -> SMCR = 0x0000; //ALWAYS 0
  //SETTING THE PRESCALER TO THE DESIRED VALUE
  TIM2 -> PSC = 319; //with this preescaler we are able to measure 38 ms in 3200 time steps. The precision is fine.
  TIM2 -> CNT = 0;
  TIM2 -> ARR = 0xFFFF; //THIS IS A STANDARD IF THERE IS NOT PWM
  TIM2 -> CCR2 = TIME_BUZZER_TOC;
  TIM3 -> CCR3 = TIME_FULL_STOP;
  TIM2 -> DIER = 0x000E; // THIS MEANS A ONE IN BIT 1, ONE IN BIT 2 AND ONE IN 3

  // EXTERNAL PIN BEHAVIOUR
  TIM2 -> CCMR1 = 0x0001; //THIS SELECTS IN THE CCMR1 A TIC IN CHANNEL 1
  TIM2 -> CCMR2 = 0x0000;
  TIM2 -> CCER = 0X000B;   //THIS SELECTS THE TIC IN CHANNEL 1
  // THE TIC IN CHANNEL 1 IS ALWAYS A TIC AND IT IS ACTIVATED
  //INITIALIZING TIMERS
  TIM2 -> CR1 |= 0x0000; //WE DO NOT WANT IT TO START YET
  TIM2 -> EGR |= 0x0001; //FOR THIS REASON WE RAISE ANY FLAGS
  TIM2 -> SR = 0;



    //TIM3 TRIGGER TOC, MEASURES 10 MICROSECONDS
  TIM3 -> CR1 = 0x0000;
  TIM3 -> CR2 = 0x0000;
  TIM3 -> SMCR = 0x0000;
  //WE DO NOT NEED PRESCALER
  TIM3 -> PSC = 0;
  TIM3 -> CNT = 0;
  TIM3 -> ARR = 0xFFFF;
  TIM3 -> CCR1 = TIME_TRIGGER_TOC;//EACH 320 STEPS WE MEASURE 10 MICROSECONDS

  TIM3 -> DIER = 0x0002;
  TIM3 -> CCMR1 = 0x0000;
  TIM3 -> CCER = 0x0000;

  TIM3 -> CR1 |= 0x0001;
  TIM3 -> EGR |= 0x0001;
  TIM3 -> SR = 0;

  //AFTER FINISHING THE INITIALIZATION, START THE TRIGGER

  GPIOD -> BSRR = (1<<2);
  Velocity_Selector();
  //ACTIVATE THE NVICS
  NVIC -> ISER[0] |= (1<<28);
  NVIC -> ISER[0] |= (1<<29);
  HAL_UART_Receive_IT(&huart1, &(buff_rx[1]), 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //send_String(1,"STARTING IN AUTOMATIC MODE:\r\n");
  //send_String(2,"STARTING IN AUTOMATIC MODE:\r\n");
  send_String("ACTIVE\r\n");
  while (1)
  {
    switch (t_value){
      case 'S':
        Full_Stop();
        break;
      case 'F':
        Full_Forward();
        break;
      case 'B':
        Full_Backward();
        break;
      case 'L':
        L_Turn();
        t_value = 'S';
        break;
      case 'R':
        R_Turn();
        t_value = 'S';
        break;
      default:
        if(distance < 10000){
                OAP();
              } else if((distance < 30000)){
                Geardown();
                if(nturns!=0){
                OAP();
                }
              } else{
                nturns=0;
                Full_Forward();
              }
        break;
    }
    if((distance >= 9500)&&(distance<=10500)){
      send_String("OBJECT AT 10 CM!\r\n");

    }else if((distance >= 29500)&&(distance<=30500)){
      send_String("OBJECT AT 20 CM!\r\n");
    }
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
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
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
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA6 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1){
  t_value = buff_rx[1];
  switch(t_value){
    case 'S':
      send_String("VEHICLE STOPPED\r\n");
      break;
    case 'F':
      send_String("GOING FORWARD\r\n");
      break;
    case 'B':
      send_String("GOING BACKWARDS\r\n");
      break;
    case 'L':
      send_String("TURNING LEFT\r\n");
      break;
    case 'R':
      send_String("TURNING RIGHT\r\n");
      break;
    case 'A':
      send_String("AUTOMATIC MODE\r\n");
      break;
  }

  HAL_UART_Receive_IT(huart1, &(buff_rx[1]), 1);

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart1){

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
