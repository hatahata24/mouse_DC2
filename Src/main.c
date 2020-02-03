/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdio.h>
#include "math.h"

#define MAIN_C_
#include "global.h"


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

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int log_cnt = 0;
int gyro_cnt = 0;
int get_cnt = 0;
//int get_speed_l[log_allay];
//int get_speed_r[log_allay];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void buzzer(int, int);
int get_adc_value(ADC_HandleTypeDef*, uint32_t);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int c) {
  if( c == '\n' ) {
    int _c = '\r';
    HAL_UART_Transmit(&huart1, &_c, 1, 1);
  }
  HAL_UART_Transmit(&huart1, &c, 1, 1);
  return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIM_OC_InitTypeDef ConfigOC;
	ConfigOC.OCMode = TIM_OCMODE_PWM1;
	ConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	ConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if(htim == &htim6){
		cnt_l = TIM5 -> CNT;
		cnt_r = TIM8 -> CNT;
		if(cnt_l > 40000) cnt_l = cnt_l - 65535;		//0=>65505の値飛び検出用
		if(cnt_r > 40000) cnt_r = cnt_r - 65535;		//0=>65505の値飛び検出用
		cnt_r = cnt_r * -1;								//a回転方向合わせ

		dist_l = dist_l + cnt_l * (DIAMETER * M_PI * 11 / 42 / 4096 / 4);
		dist_r = dist_r + cnt_r * (DIAMETER * M_PI * 11 / 42 / 4096 / 4);

		speed_l = cnt_l * (DIAMETER * M_PI * 11 / 42 / 4096 / 4) / 0.001;
		speed_r = cnt_r * (DIAMETER * M_PI * 11 / 42 / 4096 / 4) / 0.001;

		TIM5 -> CNT = 0;
		TIM8 -> CNT = 0;

		if(MF.FLAG.SPD){
			target_speed_l += accel_l * 0.001;
			target_speed_l = max(min(target_speed_l, speed_max_l), speed_min_l);
			epsilon_l = target_speed_l - speed_l;
			pulse_l = Kpl * epsilon_l;

			target_speed_r += accel_r * 0.001;
			target_speed_r = max(min(target_speed_r, speed_max_r), speed_min_r);
			epsilon_r = target_speed_r - speed_r;
			pulse_r = Kpr * epsilon_r;
		}

		log_cnt ++;

		if(log_cnt >= 5 && MF2.FLAG.LOG){
			log_cnt = 0;
			if(get_cnt < log_allay){
				get_speed_l[get_cnt] = speed_l;
				get_speed_r[get_cnt] = speed_r;
				get_omega[get_cnt] = gyro_read_z();
				get_cnt++;
			}
		}

		//gyro interrupt
//		degree_x += accel_read_x() * 0.001;
//		degree_y += accel_read_y() * 0.001;
//		degree_z += accel_read_z() * 0.001;
//		degree_x += gyro_read_x() * 0.001;
//		degree_y += gyro_read_y() * 0.001;
		degree_z += gyro_read_z() * 0.001;

		//gyro ドリフト量計算
		if(MF2.FLAG.GDRIFT){
			gyro_cnt ++;
			if(gyro_cnt >= 2)dif_omega_z += old_omega_z - gyro_read_z();
			old_omega_z = gyro_read_z();
			full_led_write1(YELLOW);
			if(gyro_cnt >= 1001) {
				MF2.FLAG.GDRIFT = 0;
				gyro_drift_value = dif_omega_z / gyro_cnt-1;
				gyro_cnt = 0;
				full_led_write1(BLUEGREEN);
			}
			degree_z = 0;
			target_degree_z = 0;
		}


		if(MF2.FLAG.ENKAI){
			target_dist = TREAD*M_PI/360*(degree_z-target_degree_z);
			if(target_dist > 0){
				target_speed_l = sqrt(2*accel_l*target_dist);
				target_speed_r = -1 * target_speed_l;
			}else{
				target_speed_l = sqrt(2*accel_l*target_dist*-1)*-1;
				target_speed_r = -1 * target_speed_l;
			}

			epsilon_l = target_speed_l - speed_l;
			pulse_l = Kpl * epsilon_l;
			epsilon_r = target_speed_r - speed_r;
			pulse_r = Kpr * epsilon_r;
		}


		if(MF.FLAG.FWALL){
			target_speed_l = (int16_t)(OFFSET_FWALL_L - ad_fl)*0.5;//0.75;
			target_speed_r = (int16_t)(OFFSET_FWALL_R - ad_fr);//*1.5;

			if(target_speed_l*target_speed_l < 2500)target_speed_l = 0;
			if(target_speed_r*target_speed_r < 2500)target_speed_r = 0;
			if(target_speed_l == 0 && target_speed_r == 0){
				MF.FLAG.FWALL = 0;
				MF.FLAG.DRV = 0;
			}

			epsilon_l = target_speed_l - speed_l;
			pulse_l = Kpl * epsilon_l;
			epsilon_r = target_speed_r - speed_r;
			pulse_r = Kpr * epsilon_r;

			pulse_l = min(max(pulse_l, -100), 100);
			pulse_r = min(max(pulse_r, -100), 100);
		}


		if(MF.FLAG.GYRO){
			target_omega_z += target_degaccel_z * 0.001;
			target_omega_z = max(min(target_omega_z, omega_max), omega_min);

			epsilon_omega = target_omega_z - gyro_read_z();

			target_speed_l = speed_G - (target_omega_z + Kp_o*epsilon_omega)/180*M_PI*TREAD/2;
			target_speed_r = speed_G + (target_omega_z + Kp_o*epsilon_omega)/180*M_PI*TREAD/2;

			epsilon_l = target_speed_l - speed_l;
			pulse_l = Kpl * epsilon_l;
			epsilon_r = target_speed_r - speed_r;
			pulse_r = Kpr * epsilon_r;
		}


		//ADchange interrupt
		uint16_t delay;
		tp = (tp+1)%3;

		switch(tp){
		  case 0:
				HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_SET); 	//L
				for(delay=0; delay<sensor_wait; delay++);
				ad_l = get_adc_value(&hadc1, ADC_CHANNEL_12);				//L
				HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);  //R
				for(delay=0; delay<sensor_wait; delay++);
				ad_r = get_adc_value(&hadc1, ADC_CHANNEL_11);				//R
				HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
			break;

		  case 1:
				HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_SET);//FL
				for(delay=0; delay<sensor_wait; delay++);
				ad_fl = get_adc_value(&hadc1, ADC_CHANNEL_13);				//FL
				HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_SET);//FR
				for(delay=0; delay<sensor_wait; delay++);
				ad_fr = get_adc_value(&hadc1, ADC_CHANNEL_10);				//FR
				HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
			break;

		  case 2:
				//
				if(MF.FLAG.WCTRL){
					int16_t dwl_tmp = 0, dwr_tmp = 0;
					if(!MF2.FLAG.V){											//a通常走行時
						dif_l = (int32_t) ad_l - base_l;
						dif_r = (int32_t) ad_r - base_r;

						if(dif_l > CTRL_BASE_L || dif_r > CTRL_BASE_R){
							if(dif_l > CTRL_BASE_L){
								dwl_tmp += CTRL_CONT_W * dif_l;					//a比例制御値を決定
								dwr_tmp += -1 * CTRL_CONT_W * dif_l;			//a比例制御値を決定
							}
							else if(dif_r > CTRL_BASE_R){
								dwl_tmp += -1 * CTRL_CONT_W * dif_r;			//a比例制御値を決定
								dwr_tmp += CTRL_CONT_W * dif_r;					//a比例制御値を決定
							}
							MF2.FLAG.WG = 1;
						}else{
							MF2.FLAG.WG = 0;
						}
					}else{														//a斜め走行時
						dif_l = (int32_t) ad_fl - BASE_FL;
						dif_r = (int32_t) ad_fr - BASE_FR;

						if(dif_l > CTRL_BASE_FL || dif_r > CTRL_BASE_FR){
							if(dif_l > CTRL_BASE_FL){
								dwl_tmp += CTRL_CONT_W * 0.1 * dif_l;				//a比例制御値を決定
								dwr_tmp += -1 * CTRL_CONT_W * 0.1 * dif_l;			//a比例制御値を決定
							}
							else if(dif_r > CTRL_BASE_FR){
								dwl_tmp += -1 * CTRL_CONT_W * 0.2 * dif_r;			//a比例制御値を決定
								dwr_tmp += CTRL_CONT_W * 0.2 * dif_r;				//a比例制御値を決定
							}
							MF2.FLAG.WG = 1;
						}else{
							MF2.FLAG.WG = 0;
						}
					}
					dwl = max(min(CTRL_MAX_W, dwl_tmp), -1 * CTRL_MAX_W);
					dwr = max(min(CTRL_MAX_W, dwr_tmp), -1 * CTRL_MAX_W);
				}else{
					//a制御フラグがなければ壁制御値0
					dwl = dwr = 0;
				}

				if(MF.FLAG.GCTRL){
/*					int16_t dg_tmp = 0;
					dif_g = (int32_t) gyro_read_z() - target_omega_z;		//a角速度閾値越え制御

					if(CTRL_BASE_G < dif_g){					//a角速度変化量が基準よりも大きい時(左回転が発生時)
						dg_tmp += CTRL_CONT_G * dif_g;			//a比例制御値を決定
					}
					else if(-1*CTRL_BASE_G > dif_g){			//a角速度変化量が負の基準よりも小さい時(右回転が発生時)
						dg_tmp += CTRL_CONT_G * dif_g;			//a比例制御値を決定
					}
					dg = max(min(CTRL_MAX_G, dg_tmp), -1 * CTRL_MAX_G);
					dgl = dg;
					dgr = -1*dg;
				}else{
					//a制御フラグがなければ壁制御値0
					dgl = dgr = 0;
				}

					dg = CTRL_CONT_G * gyro_read_z();			//a角速度制御
					dg = CTRL_CONT_G * degree_z;				//a角度制御
*/
					dg = CTRL_CONT_G * (target_degree_z - degree_z);		//a角度制御(目標角度はスタートを0度とし、旋回量と対応付け)

					dg = max(min(CTRL_MAX_G, dg), -1 * CTRL_MAX_G);
					dgl = -1*dg;
					dgr = dg;
				}else{
					//a制御フラグがなければジャイロ制御値0
					dgl = dgr = 0;
				}
				break;
		}


		if(MF.FLAG.DRV){
			if(!MF2.FLAG.WG){
				pulse_l = pulse_l + dgl + dwl;
				pulse_r = pulse_r + dgr + dwr;
//				pulse_l = pulse_l + dgl;
//				pulse_r = pulse_r + dgr;
			}else{
				pulse_l = pulse_l + dgl + dwl;
				pulse_r = pulse_r + dgr + dwr;
//				pulse_l = pulse_l + dwl;
//				pulse_r = pulse_r + dwr;
			}
			pulse_l = min(max(pulse_l, -1000), 1000);
			pulse_r = min(max(pulse_r, -1000), 1000);

			if(pulse_l >= 0){
				drive_dir(0, 0);
				ConfigOC.Pulse = pulse_l;
				HAL_TIM_PWM_ConfigChannel(&htim3, &ConfigOC, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
			}
			else if(pulse_l < 0){
				drive_dir(0, 1);
				ConfigOC.Pulse = -pulse_l;
				HAL_TIM_PWM_ConfigChannel(&htim3, &ConfigOC, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
			}

			if(pulse_r >= 0){
				drive_dir(1, 0);
				ConfigOC.Pulse = pulse_r;
				HAL_TIM_PWM_ConfigChannel(&htim12, &ConfigOC, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
			}
			else if(pulse_r < 0){
				drive_dir(1, 1);
				ConfigOC.Pulse = -pulse_r;
				HAL_TIM_PWM_ConfigChannel(&htim12, &ConfigOC, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
			}
		}else{
			drive_dir(0, 2);
			drive_dir(1, 2);
		}


		//wall check
		//----look L----
		if(ad_l > WALL_BASE_L){
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
		}
		//----look R----
		if(ad_r > WALL_BASE_R){
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		}
		//----look FL----
		if(ad_fl > WALL_BASE_FL){
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
		}
		//----look FR----
		if(ad_fr > WALL_BASE_FR){
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		}


		//battery check
		if(HAL_GPIO_ReadPin(VOL_CHECK_GPIO_Port, VOL_CHECK_Pin) == GPIO_PIN_RESET) {	//2.1V以下で赤ランプ点灯=>LiPoが約7Vを下回るとランプ点灯
		   HAL_GPIO_WritePin(VAT_ALERT_GPIO_Port, VAT_ALERT_Pin, GPIO_PIN_SET);
		} else {
		   HAL_GPIO_WritePin(VAT_ALERT_GPIO_Port, VAT_ALERT_Pin, GPIO_PIN_RESET);
		}


		//fail safe
		if(degree_z >= target_degree_z+270 || degree_z <= target_degree_z-270 || dist_r > 500 || dist_l > 500){	//270度以上回転発生でFail Safe
			while(1){
			   drive_dir(0, 2);
			   drive_dir(1, 2);
			   full_led_write1(RED);
		   }
		}
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
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  drive_init();
  gyro_init();
  search_init();
  sensor_init();

  printf("*** Welcome to WMMC ! ***\n");

  setbuf(stdout, NULL);
  HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim6);

  int mode = 0;
  printf("Mode : %d\n", mode);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  led_write(mode & 0b001, mode & 0b010, mode & 0b100);
	  if(dist_r >= 20){
		  mode++;
		  dist_r = 0;
		  if(mode > 7){
			  mode = 0;
		  }
		  printf("Mode : %d\n", mode);
	  }
	  if(dist_r <= -20){
		  mode--;
		  dist_r = 0;
		  if(mode < 0){
			  mode = 7;
		  }
		  printf("Mode : %d\n", mode);
	  }
	  if(dist_l <= -20){
		  dist_l = 0;
		  switch(mode){

			  case 0:
				  HAL_Delay(5000);
				  break;

			  case 1:
				  //----a超新地走行----
				  printf("Simple Run.\n");
				  //MF.FLAG.WEDGE = 1;
	//		  		  simple_run();
				  perfect_run();
				  break;

			  case 2:
				  //----aスラローム走行----
				  printf("slalom Run.\n");
				  //MF.FLAG.WEDGE = 1;
				  slalom_run();
				  break;

			  case 3:
				  //----aスラローム走行&pass圧縮----
				  printf("First Run. (Slalom)\n");
				  pass_test();
				  break;

			  case 4:
				  //----aテストモード選択----
				  test_select();
				  break;

			  case 5:
				  //----sensor check----
				  printf("Sensor Check.\n");
				  sensor_test();
				  break;

			  case 6:
				  //----pitagola sound----
				  while(ad_fl <= WALL_BASE_FL){
					  led_write(1, 1, 1);
					  HAL_Delay(200);
					  led_write(0, 0, 0);
					  HAL_Delay(200);
				  }
				  HAL_Delay(200);
	/*		  		  for(int i=0; i<pita; i++){
					  buzzer(pitagola[i][0], pitagola[i][1]);
				  }
				  for(int i=0; i<m_start; i++){
					  buzzer(mario_start[i][0], mario_start[i][1]);
				  }
				  for(int i=0; i<m_coin; i++){
					  buzzer(mario_coin[i][0], mario_coin[i][1]);
				  }
				  for(int i=0; i<m_select; i++){
					  buzzer(mario_select[i][0], mario_select[i][1]);
				  }
				  HAL_Delay(2000);
				  for(int i=0; i<m_ok; i++){
					  buzzer(mario_ok[i][0], mario_ok[i][1]);
				  }
	*/				  HAL_Delay(2000);
				  for(int i=0; i<m_goal; i++){
					  buzzer(mario_goal[i][0], mario_goal[i][1]);
				  }
				  break;

			  case 7:
				  //----a本番走行用----
				  //MF.FLAG.WEDGE = 1;
	/*		  		  for(int i=0; i<m_select; i++){
					  buzzer(mario_select[i][0], mario_select[i][1]);
				  }
	*///		  		  perfect_run();
				  perfect_slalom();
				  break;
		  }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  sConfig.Channel = ADC_CHANNEL_10;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FLED2_RED_Pin|FLED2_GREEN_Pin|FLED2_BLUE_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, IR_L_Pin|IR_FL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED6_Pin|VAT_ALERT_Pin|LED7_Pin|LED3_Pin 
                          |LED2_Pin|LED1_Pin|IR_FR_Pin|IR_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_L_CCW_Pin|MOTOR_L_CW_Pin|MOTOR_L_R_STBY_Pin|MOTOR_R_CW_Pin 
                          |LED5_Pin|MOTOR_R_CCW_Pin|FLED1_BLUE_Pin|FLED1_RED_Pin 
                          |FLED1_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FLED2_RED_Pin FLED2_GREEN_Pin FLED2_BLUE_Pin LED4_Pin */
  GPIO_InitStruct.Pin = FLED2_RED_Pin|FLED2_GREEN_Pin|FLED2_BLUE_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_L_Pin IR_FL_Pin */
  GPIO_InitStruct.Pin = IR_L_Pin|IR_FL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LED6_Pin VAT_ALERT_Pin LED7_Pin LED3_Pin 
                           LED2_Pin LED1_Pin IR_FR_Pin IR_R_Pin */
  GPIO_InitStruct.Pin = LED6_Pin|VAT_ALERT_Pin|LED7_Pin|LED3_Pin 
                          |LED2_Pin|LED1_Pin|IR_FR_Pin|IR_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_L_CCW_Pin MOTOR_L_CW_Pin MOTOR_L_R_STBY_Pin MOTOR_R_CW_Pin 
                           LED5_Pin MOTOR_R_CCW_Pin FLED1_BLUE_Pin FLED1_RED_Pin 
                           FLED1_GREEN_Pin */
  GPIO_InitStruct.Pin = MOTOR_L_CCW_Pin|MOTOR_L_CW_Pin|MOTOR_L_R_STBY_Pin|MOTOR_R_CW_Pin 
                          |LED5_Pin|MOTOR_R_CCW_Pin|FLED1_BLUE_Pin|FLED1_RED_Pin 
                          |FLED1_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VOL_CHECK_Pin */
  GPIO_InitStruct.Pin = VOL_CHECK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VOL_CHECK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PUSH_IN_Pin */
  GPIO_InitStruct.Pin = PUSH_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PUSH_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void buzzer(int sound, int length){
	TIM_OC_InitTypeDef ConfigOC;
	ConfigOC.OCMode = TIM_OCMODE_PWM1;
	ConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	ConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	hz = 1000000 / sound;
	TIM3 -> ARR = hz;
    ConfigOC.Pulse = hz / 10;
    HAL_TIM_PWM_ConfigChannel(&htim4, &ConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	HAL_Delay(length);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}


int get_adc_value(ADC_HandleTypeDef *hadc, uint32_t channel){

  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset = 0;

  HAL_ADC_ConfigChannel(hadc, &sConfig);

  HAL_ADC_Start(hadc);                    //
  HAL_ADC_PollForConversion(hadc, 100);   //wait for ADC
  return HAL_ADC_GetValue(hadc);          //
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
