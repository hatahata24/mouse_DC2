
#include "global.h"


void sensor_init(void){
	tp = 0;
	ad_l = ad_r = ad_fr = ad_fl = 0;
	base_l = base_r = 0;
}


uint8_t get_base(){
	uint8_t res = 1;									//for return

	base_l = ad_l;										//sensor value base L
	base_r = ad_r;										//sensor value base R

	return res;											//
}


void get_wall_info(){

	//----reset----
	wall_info = 0x00;									//wall
	//----look forward----
	if(ad_fr > WALL_BASE_FR || ad_fl > WALL_BASE_FL){
		wall_info |= 0x88;								//forward check
	}
	//----look right----
	if(ad_r > WALL_BASE_R){
		wall_info |= 0x44;								//right check
	}
	//----look left----
	if(ad_l > WALL_BASE_L){
		wall_info |= 0x11;								//light check
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//led_write
//aモード選択用LEDのON-OFF
//a引数：led1(0=>OFF, 1=>ON), led2(0=>OFF, 1=>ON), led3(0=>OFF, 1=>ON)
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void led_write(uint8_t led3, uint8_t led2, uint8_t led1){
	if(led1) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	if(led2) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	if(led3) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//full_led_write1
//a頭部フルカラーLEDの色選択
//a引数：fulled(0=>OFF, 1=>a赤, 2=>緑, 3=>青, 4=>青緑, 5=>紫, 6=>黄, 7=>白)
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void full_led_write1(uint8_t fulled){
	if(fulled == 0){
	    HAL_GPIO_WritePin(FLED1_RED_GPIO_Port, FLED1_RED_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED1_GREEN_GPIO_Port, FLED1_GREEN_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED1_BLUE_GPIO_Port, FLED1_BLUE_Pin, GPIO_PIN_SET);
	}
	else if(fulled == 1){
	    HAL_GPIO_WritePin(FLED1_RED_GPIO_Port, FLED1_RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED1_GREEN_GPIO_Port, FLED1_GREEN_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED1_BLUE_GPIO_Port, FLED1_BLUE_Pin, GPIO_PIN_SET);
	}
	else if(fulled == 2){
	    HAL_GPIO_WritePin(FLED1_RED_GPIO_Port, FLED1_RED_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED1_GREEN_GPIO_Port, FLED1_GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED1_BLUE_GPIO_Port, FLED1_BLUE_Pin, GPIO_PIN_SET);
	}
	else if(fulled == 3){
	    HAL_GPIO_WritePin(FLED1_RED_GPIO_Port, FLED1_RED_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED1_GREEN_GPIO_Port, FLED1_GREEN_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED1_BLUE_GPIO_Port, FLED1_BLUE_Pin, GPIO_PIN_RESET);
	}
	else if(fulled == 4){
	    HAL_GPIO_WritePin(FLED1_RED_GPIO_Port, FLED1_RED_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED1_GREEN_GPIO_Port, FLED1_GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED1_BLUE_GPIO_Port, FLED1_BLUE_Pin, GPIO_PIN_RESET);
	}
	else if(fulled == 5){
	    HAL_GPIO_WritePin(FLED1_RED_GPIO_Port, FLED1_RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED1_GREEN_GPIO_Port, FLED1_GREEN_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED1_BLUE_GPIO_Port, FLED1_BLUE_Pin, GPIO_PIN_RESET);
	}
	else if(fulled == 6){
	    HAL_GPIO_WritePin(FLED1_RED_GPIO_Port, FLED1_RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED1_GREEN_GPIO_Port, FLED1_GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED1_BLUE_GPIO_Port, FLED1_BLUE_Pin, GPIO_PIN_SET);
	}
	else if(fulled == 7){
	    HAL_GPIO_WritePin(FLED1_RED_GPIO_Port, FLED1_RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED1_GREEN_GPIO_Port, FLED1_GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED1_BLUE_GPIO_Port, FLED1_BLUE_Pin, GPIO_PIN_RESET);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//full_led_write2
//a大フルカラーLEDの色選択
//a引数：fulled(0=>OFF, 1=>a赤, 2=>緑, 3=>青, 4=>青緑, 5=>紫, 6=>黄, 7=>白)
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void full_led_write2(uint8_t fulled){
	if(fulled == 0){
	    HAL_GPIO_WritePin(FLED2_RED_GPIO_Port, FLED2_RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED2_GREEN_GPIO_Port, FLED2_GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED2_BLUE_GPIO_Port, FLED2_BLUE_Pin, GPIO_PIN_RESET);
	}
	else if(fulled == 1){
	    HAL_GPIO_WritePin(FLED2_RED_GPIO_Port, FLED2_RED_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED2_GREEN_GPIO_Port, FLED2_GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED2_BLUE_GPIO_Port, FLED2_BLUE_Pin, GPIO_PIN_RESET);
	}
	else if(fulled == 2){
	    HAL_GPIO_WritePin(FLED2_RED_GPIO_Port, FLED2_RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED2_GREEN_GPIO_Port, FLED2_GREEN_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED2_BLUE_GPIO_Port, FLED2_BLUE_Pin, GPIO_PIN_RESET);
	}
	else if(fulled == 3){
	    HAL_GPIO_WritePin(FLED2_RED_GPIO_Port, FLED2_RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED2_GREEN_GPIO_Port, FLED2_GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED2_BLUE_GPIO_Port, FLED2_BLUE_Pin, GPIO_PIN_SET);
	}
	else if(fulled == 4){
	    HAL_GPIO_WritePin(FLED2_RED_GPIO_Port, FLED2_RED_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED2_GREEN_GPIO_Port, FLED2_GREEN_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED2_BLUE_GPIO_Port, FLED2_BLUE_Pin, GPIO_PIN_SET);
	}
	else if(fulled == 5){
	    HAL_GPIO_WritePin(FLED2_RED_GPIO_Port, FLED2_RED_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED2_GREEN_GPIO_Port, FLED2_GREEN_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(FLED2_BLUE_GPIO_Port, FLED2_BLUE_Pin, GPIO_PIN_SET);
	}
	else if(fulled == 6){
	    HAL_GPIO_WritePin(FLED2_RED_GPIO_Port, FLED2_RED_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED2_GREEN_GPIO_Port, FLED2_GREEN_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED2_BLUE_GPIO_Port, FLED2_BLUE_Pin, GPIO_PIN_RESET);
	}
	else if(fulled == 7){
	    HAL_GPIO_WritePin(FLED2_RED_GPIO_Port, FLED2_RED_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED2_GREEN_GPIO_Port, FLED2_GREEN_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(FLED2_BLUE_GPIO_Port, FLED2_BLUE_Pin, GPIO_PIN_SET);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//sensor_test
//a壁センサーとジャイロセンサーの値確認
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void sensor_test(){

	int mode = 0;
	printf("Mode : %d\n", mode);

	while(1){
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
			  drive_ready();

			  switch(mode){
				case 0:
					get_base();
					break;
				case 1:
					  //----Wall sensor check----
					  printf("Wall Sensor Check.\n");
					  while(1){
						  get_wall_info();
						  led_write(wall_info & 0x11, wall_info & 0x88, wall_info & 0x44);
						  printf("ad_l : %d, ad_fl : %d, ad_fr : %d, ad_r : %d\n", ad_l, ad_fl, ad_fr, ad_r);
						  HAL_Delay(333);
						}
					break;
				case 2:
					//----Gyro sensor check----
					printf("Gyro Sensor Check.\n");
					int accel_x, accel_y, accel_z;
					int gyro_x, gyro_y, gyro_z;
					int deg_x, deg_y, deg_z;
					  while(1){
						  accel_x = accel_read_x();
						  accel_y = accel_read_y();
						  accel_z = accel_read_z();
						  gyro_x = gyro_read_x();
						  gyro_y = gyro_read_y();
						  gyro_z = gyro_read_z();
						  deg_x = degree_x;
						  deg_y = degree_y;
						  deg_z = degree_z;

						  //printf("Accel x: %3d, y: %3d, z: %3d\n", accel_x, accel_y, accel_z);
						  printf("Gyro  x: %3d, y: %3d, z: %3d\n", gyro_x, gyro_y, gyro_z);
						  printf("Deg   x: %3d, y: %3d, z: %3d\n", deg_x, deg_y, deg_z);
						  HAL_Delay(111);
						}
					break;
				case 3:
					break;
				case 4:
					break;
				case 5:
					break;
				case 6:
					break;
				case 7:
					break;
			  }
			  dist_l = 0;
		}
	}
}

