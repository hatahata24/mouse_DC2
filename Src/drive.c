
#include "global.h"
#include "math.h"

//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_init
// a走行系の変数の初期化
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_init(void){
	MF.FLAGS = 0;
	MF2.FLAGS2 = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_ready
// a走行前のLED点滅&ジャイロのドリフト計算
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_ready(void){
	  while(ad_fl <= WALL_BASE_FL && ad_fr <= WALL_BASE_FR){
		  led_write(1, 1, 1);
		  HAL_Delay(200);
		  led_write(0, 0, 0);
		  HAL_Delay(200);
	  }
	  MF2.FLAG.GDRIFT = 1;
	  HAL_Delay(2000);
	  degree_z = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_start
// a走行開始前に走行距離と機体角度を初期化
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_start(void){
	dist_l = dist_r = 0;		//a走行距離の初期化
	if(!MF2.FLAG.HACCEL) target_speed_l = target_speed_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	MF.FLAG.SPD = 1;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_stop
// a走行を終了する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_stop(void){
	dist_l = dist_r = 0;		//a走行距離の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 0;
	MF.FLAG.SPD = 0;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_break
// a走行中に急停止する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_break(void){
	target_speed_l = target_speed_r = 0;		//a目標速度の初期化
	accel_l = accel_r = 0;		//a加速度の初期化
	pulse_l = pulse_r = 0;		//aモータ出力の初期化
	MF.FLAG.DRV = 1;
	MF.FLAG.SPD = 1;
	control_stop();
	full_led_write1(RED);
	HAL_Delay(100);
	MF.FLAG.DRV = 0;
	MF.FLAG.SPD = 0;
	full_led_write1(BLUE);
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//control_start
// wallとgyroの姿勢制御を開始する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void control_start(){
	MF.FLAG.WCTRL = 1;										//wall制御を有効にする
	if(run_mode == 5 && MF2.FLAG.TEMP)MF.FLAG.WCTRL = 0;	//a高速走行時に壁制御を無効化し、ジャイロ制御のみに切り替え
	MF.FLAG.GCTRL = 1;										//gyro制御を有効にする
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//control_stop
// wallとgyroの姿勢制御を停止する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void control_stop(){
	MF.FLAG.WCTRL = 0;										//wall制御を無効にする
	MF.FLAG.GCTRL = 0;										//gyro制御を無効にする
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//drive_dir
// wheel turn dir for each wheel
// a引数:1車輪選択(0=>L, 1=>R), 2回転方向選択(0=>CW, 1=>CWW, 2=>ShortBrake, 3=>free)
// a戻り値: なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_dir(uint8_t wheel, uint8_t dir){
	if(wheel == 0){
		if(dir == 0){
		    HAL_GPIO_WritePin(MOTOR_L_CW_GPIO_Port, MOTOR_L_CW_Pin, GPIO_PIN_SET);				//L_CW
		    HAL_GPIO_WritePin(MOTOR_L_CCW_GPIO_Port, MOTOR_L_CCW_Pin, GPIO_PIN_RESET);			//L_CCW
		    HAL_GPIO_WritePin(MOTOR_L_R_STBY_GPIO_Port, MOTOR_L_R_STBY_Pin, GPIO_PIN_SET);		//STBY
		}else if(dir == 1){
		    HAL_GPIO_WritePin(MOTOR_L_CW_GPIO_Port, MOTOR_L_CW_Pin, GPIO_PIN_RESET);			//L_CW
		    HAL_GPIO_WritePin(MOTOR_L_CCW_GPIO_Port, MOTOR_L_CCW_Pin, GPIO_PIN_SET);			//L_CCW
		    HAL_GPIO_WritePin(MOTOR_L_R_STBY_GPIO_Port, MOTOR_L_R_STBY_Pin, GPIO_PIN_SET);		//STBY
		}else if(dir == 2){
		    HAL_GPIO_WritePin(MOTOR_L_CW_GPIO_Port, MOTOR_L_CW_Pin, GPIO_PIN_SET);				//L_CW
		    HAL_GPIO_WritePin(MOTOR_L_CCW_GPIO_Port, MOTOR_L_CCW_Pin, GPIO_PIN_SET);			//L_CCW
		    HAL_GPIO_WritePin(MOTOR_L_R_STBY_GPIO_Port, MOTOR_L_R_STBY_Pin, GPIO_PIN_SET);		//STBY
		}else{
		    HAL_GPIO_WritePin(MOTOR_L_R_STBY_GPIO_Port, MOTOR_L_R_STBY_Pin, GPIO_PIN_SET);		//STBY
		}
	}else{
		if(dir == 0){
		    HAL_GPIO_WritePin(MOTOR_R_CW_GPIO_Port, MOTOR_R_CW_Pin, GPIO_PIN_SET);				//R_CW
		    HAL_GPIO_WritePin(MOTOR_R_CCW_GPIO_Port, MOTOR_R_CCW_Pin, GPIO_PIN_RESET);			//R_CCW
		    HAL_GPIO_WritePin(MOTOR_L_R_STBY_GPIO_Port, MOTOR_L_R_STBY_Pin, GPIO_PIN_SET);		//STBY
		}else if(dir == 1){
		    HAL_GPIO_WritePin(MOTOR_R_CW_GPIO_Port, MOTOR_R_CW_Pin, GPIO_PIN_RESET);			//R_CW
		    HAL_GPIO_WritePin(MOTOR_R_CCW_GPIO_Port, MOTOR_R_CCW_Pin, GPIO_PIN_SET);			//R_CCW
		    HAL_GPIO_WritePin(MOTOR_L_R_STBY_GPIO_Port, MOTOR_L_R_STBY_Pin, GPIO_PIN_SET);		//STBY
		}else if(dir == 2){
		    HAL_GPIO_WritePin(MOTOR_R_CW_GPIO_Port, MOTOR_R_CW_Pin, GPIO_PIN_SET);				//R_CW
		    HAL_GPIO_WritePin(MOTOR_R_CCW_GPIO_Port, MOTOR_R_CCW_Pin, GPIO_PIN_SET);			//R_CCW
		    HAL_GPIO_WritePin(MOTOR_L_R_STBY_GPIO_Port, MOTOR_L_R_STBY_Pin, GPIO_PIN_SET);		//STBY
		}else{
		    HAL_GPIO_WritePin(MOTOR_L_R_STBY_GPIO_Port, MOTOR_L_R_STBY_Pin, GPIO_PIN_SET);		//STBY
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//run_select
// a走行速度を選択する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_select(){
	full_led_write1(YELLOW);
	int mode = 0;

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
		if(ad_fl >= WALL_BASE_FL && ad_fr >= WALL_BASE_FR){
			  run_mode = mode;
			  full_led_write1(BLUEGREEN);
			  break;
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveA
// a指定距離、指定加速度で加速走行する
// a引数1：accel_p 加速度, 引数2：speed_min_p 最低速度, 引数3：speed_max_p 最高速度, 引数4：dist 走行距離
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveA(uint16_t accel_p, uint16_t speed_min_p, uint16_t speed_max_p, uint16_t dist){

	speed_min_l = speed_min_r = speed_min_p;
	speed_max_l = speed_max_r = speed_max_p;
	accel_l = accel_r = accel_p;							//a引数の各パラメータをグローバル変数化
	if(MF2.FLAG.HACCEL)target_speed_l = target_speed_r = speed_min_p;

	drive_start();											//a走行開始

	//----a走行----
	while((dist_l < dist) || (dist_r < dist));				//a左右のモータが指定距離以上進むまで待機
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveD
// a指定距離、指定減速度で減速走行する
// a引数1：accel_p 加速度, 引数2：speed_min_p 最低速度, 引数3：speed_max_p 最高速度, 引数4：dist 走行距離
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveD(int16_t accel_p, uint16_t speed_min_p, uint16_t speed_max_p, uint16_t dist){

	float speed_0 = (target_speed_l + target_speed_r) / 2;					//a等速走行距離を計算するためにmain.cより参照
	speed_min_l = speed_min_r = speed_min_p;
	speed_max_l = speed_max_r = speed_max_p;
	accel_l = accel_r = accel_p;											//a引数の各パラメータをグローバル変数化

	int16_t c_dist = dist - (speed_min_l*speed_min_l  - speed_0*speed_0)/(2*accel_l);			//a等速走行距離 = 総距離 - 減速に必要な距離

	accel_l = accel_r = 0;
	dist_l = dist_r = 0;
	if(c_dist > 0){
		//----a等速走行----
		while((dist_l < c_dist) || (dist_r < c_dist));						//a左右のモータが等速分の距離以上進むまで待機
	}
	accel_l = accel_r = accel_p;
	//----a減速走行----
	while((dist_l < dist) && (dist_r < dist));								//a左右のモータが減速分の距離以上進むまで待機

	if(!MF2.FLAG.HACCEL)drive_stop();										//a走行停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveU
// a指定距離分等速走行して停止する
// a引数1：dist …… 走行距離
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveU(uint16_t dist){

	accel_l = accel_r = 0;									//a等速走行のため加速度は0
	dist_l = dist_r = 0;

	//----a走行----
	while((dist_l < dist) || (dist_r < dist)){				//a左右のモータが指定パルス以上進むまで待機
		if(MF.FLAG.WEDGE){
			if(ad_l < WALL_BASE_L-40 || ad_r < WALL_BASE_R-40){
				while((dist_l < W_DIST) || (dist_r < W_DIST));	//a左右のモータが壁切れ用指定距離以上進むまで待機
			break;
			}
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC
// a指定距離分デフォルト速度で走行して停止する
// a引数1：dist …… 走行距離
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(uint16_t dist){

	speed_min_l = speed_min_r = 150;
	speed_max_l = speed_max_r = 150;
	accel_l = accel_r = 0;									//a等速走行のため加速度は0

	drive_start();											//a走行開始
	//====a回転====
	while((dist_l < dist) || (dist_r < dist));				//a左右のモータが定速分の距離以上進むまで待機

	drive_stop();											//a走行停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//driveC2
//a指定距離分デフォルト逆回転速度で走行して停止する
//a引数1：dist …… 走行距離
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC2(uint16_t dist){

	speed_min_l = speed_min_r = -250;
	speed_max_l = speed_max_r = -250;
	accel_l = accel_r = 0;									//a等速走行のため加速度は0

	drive_start();											//a走行開始
	//====a回転====
	while((dist_l > (-1*dist)) || (dist_r > (-1*dist)));	//a左右のモータが定速分の逆走距離以上進むまで待機

	drive_stop();											//a走行停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalomF
//aスラロームの前オフセット部分
//a引数1：
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalomF(int16_t accel_p, int16_t speed_p, uint8_t dist_p, uint16_t wall_fl, uint16_t wall_fr){
	MF.FLAG.GYRO = 0;
	MF.FLAG.SPD = 1;

	accel_l = accel_r = accel_p;
	speed_max_l = speed_max_r = speed_p;

	control_start();
	dist_l = dist_r = 0;
	while((dist_l+dist_r)/2 < dist_p){
		if(ad_fl > wall_fl || ad_fr > wall_fr){
			full_led_write2(RED);
			break;
		}
	}
	drive_stop();
	control_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalomR
//aスラロームの旋回部分
//a引数1：
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalomR(int32_t degaccel_p, int16_t omega_p, int16_t degree_p, int16_t speed_p){
	MF.FLAG.GYRO = 1;
	full_led_write2(YELLOW);
	target_degaccel_z = degaccel_p;
	target_omega_z = 0;
	speed_G = speed_p;

	int16_t c_degree;

	if(omega_p < 0){
		omega_min = omega_p;
		c_degree =  omega_min * omega_min / target_degaccel_z / 2;
	}else{
		omega_max = omega_p;
		c_degree =  omega_max * omega_max / target_degaccel_z / 2;
	}

	MF.FLAG.DRV = 1;
	if(omega_p < 0){
		while(degree_z > target_degree_z+c_degree);
	}else{
		while(degree_z < target_degree_z+c_degree);
	}

	target_degaccel_z = 0;

	if(MF2.FLAG.TEMP){
		if(omega_p < 0){
			while(degree_z > target_degree_z+degree_p);
		}else{
			while(degree_z < target_degree_z+degree_p);
		}
	}else{
		if(omega_p < 0){
			while(degree_z > target_degree_z+(degree_p-c_degree-4));
		}else{
			while(degree_z < target_degree_z+(degree_p-c_degree+4));
		}

		target_degaccel_z = -degaccel_p;

		if(omega_p < 0){
			while(degree_z > target_degree_z+degree_p);
		}else{
			while(degree_z < target_degree_z+degree_p);
		}
	}

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalomB
//aスラロームの後オフセット部分
//a引数1：
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalomB(int16_t accel_p, int16_t speed_p, uint8_t dist_p){
	full_led_write2(BLUEGREEN);
	MF.FLAG.GYRO = 0;
	MF.FLAG.SPD = 1;

	accel_l = accel_r = accel_p;
	speed_max_l = speed_max_r = speed_p;

	control_start();
	dist_l = dist_r = 0;
	while((dist_l+dist_r)/2 < dist_p);
}



//+++++++++++++++++++++++++++++++++++++++++++++++
//set_position
//a機体の尻を壁に当てて場所を区画中央に合わせる
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_position(){

  driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
  degree_z = target_degree_z;
  start_mode = 0;
  start_sectionA();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//set_positionF
//a前壁との距離を測定し、場所を区画中央に合わせる
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_positionF(){

	full_led_write1(RED);
	HAL_Delay(100);

	MF.FLAG.DRV = 1;
//	MF.FLAG.FWALL = 1;
	MF.FLAG.GCTRL = 1;
//	while(MF.FLAG.FWALL);

	MF.FLAG.GCTRL = 0;
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//start_sectionA
// スタート区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void start_sectionA(void){

	control_start();
	if(run_mode == 1){
		if(start_mode == 0){
			driveA(2000, SPEED_MIN, SPEED_1, SEC_START);					//aスタート区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 1){
			driveA(2000, SPEED_MIN, SPEED_1, SEC_HALF);					//a半区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 2){
			driveA(2000, SPEED_MIN, SPEED_1, SEC_START_HALF);				//aスタート半区画分加速しながら走行。走行後は停止しない
		}
	}else if(run_mode == 2){
		if(start_mode == 0){
			driveA(4000, SPEED_MIN, SPEED_2, SEC_START);				//aスタート区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 1){
			driveA(4000, SPEED_MIN, SPEED_2, SEC_HALF);				//a半区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 2){
			driveA(4000, SPEED_MIN, SPEED_2, SEC_START_HALF);			//aスタート半区画分加速しながら走行。走行後は停止しない
		}
	}else if(run_mode == 3){
		if(start_mode == 0){
			driveA(6000, SPEED_MIN, SPEED_3, SEC_START);					//aスタート区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 1){
			driveA(6000, SPEED_MIN, SPEED_3, SEC_HALF);					//a半区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 2){
			driveA(6000, SPEED_MIN, SPEED_3, SEC_START_HALF);			//aスタート半区画分加速しながら走行。走行後は停止しない
		}
	}else if(run_mode == 4){
		if(start_mode == 0){
			driveA(8000, SPEED_MIN, SPEED_4, SEC_START);					//aスタート区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 1){
			driveA(8000, SPEED_MIN, SPEED_4, SEC_HALF);					//a半区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 2){
			driveA(10000, SPEED_MIN, SPEED_4, SEC_START_HALF);			//aスタート半区画分加速しながら走行。走行後は停止しない
		}
	}else if(run_mode == 5){
		if(start_mode == 0){
			driveA(10000, SPEED_MIN, SPEED_5, SEC_START);					//aスタート区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 1){
			driveA(10000, SPEED_MIN, SPEED_5, SEC_HALF);					//a半区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 2){
			driveA(15000, SPEED_MIN, SPEED_5, SEC_START_HALF);			//aスタート半区画分加速しながら走行。走行後は停止しない
		}
	}else if(run_mode == 6){
		if(start_mode == 0){
			driveA(10000, SPEED_MIN, SPEED_5, SEC_START);					//aスタート区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 1){
			driveA(10000, SPEED_MIN, SPEED_5, SEC_HALF);					//a半区画分加速しながら走行。走行後は停止しない
		}else if(start_mode == 2){
			driveA(25000, SPEED_MIN, SPEED_5, SEC_START_HALF);			//aスタート半区画分加速しながら走行。走行後は停止しない
		}
	}
	start_mode = 1;
	if(!MF.FLAG.SCND)get_wall_info();								//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionA
// a半区画分加速しながら走行する
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA(void){

	control_start();
	if(run_mode == 1){
		driveA(4000, SPEED_MIN, SPEED_1, SEC_HALF);						//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 2){
		driveA(4000, SPEED_MIN, SPEED_2, SEC_HALF);					//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 3){
		driveA(6000, SPEED_MIN, SPEED_3, SEC_HALF);						//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 4){
		driveA(8000, SPEED_MIN, SPEED_4, SEC_HALF);						//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 5){
		driveA(10000, SPEED_MIN, SPEED_5, SEC_HALF);						//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 6){
		driveA(10000, SPEED_MIN, SPEED_6, SEC_HALF);						//半区画分加速しながら走行。走行後は停止しない
	}
	if(!MF.FLAG.SCND)get_wall_info();										//壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionD
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD(void){
	full_led_write1(BLUE);

	control_start();
	if(run_mode == 1){
		driveD(-4000, SPEED_MIN, SPEED_1, SEC_HALF);						//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 2){
		driveD(-4000, SPEED_MIN, SPEED_2, SEC_HALF);					//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 3){
		driveD(-6000, SPEED_MIN, SPEED_3, SEC_HALF);						//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 4){
		driveD(-8000, SPEED_MIN, SPEED_4, SEC_HALF);				//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 5){
		driveD(-10000, SPEED_MIN, SPEED_5, SEC_HALF);				//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 6){
		driveD(-10000, SPEED_MIN, SPEED_6, SEC_HALF);				//半区画分指定減速度で減速走行。走行後は停止する
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionVA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionVA(void){

	control_start();
	if(run_mode == 1){
		driveA(4000, SPEED_MIN, SPEED_1, SEC_HALF_V);						//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 2){
		driveA(4000, SPEED_MIN, SPEED_2, SEC_HALF_V);					//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 3){
		driveA(6000, SPEED_MIN, SPEED_3, SEC_HALF_V);						//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 4){
		driveA(8000, SPEED_MIN, SPEED_4, SEC_HALF_V);						//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 5){
		driveA(10000, SPEED_MIN, SPEED_5, SEC_HALF_V);						//半区画分加速しながら走行。走行後は停止しない
	}else if(run_mode == 6){
		driveA(10000, SPEED_MIN, SPEED_6, SEC_HALF_V);						//半区画分加速しながら走行。走行後は停止しない
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionVD
// 半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionVD(void){
	full_led_write1(BLUE);

	control_start();
	if(run_mode == 1){
		driveD(-4000, SPEED_MIN, SPEED_1, SEC_HALF_V);						//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 2){
		driveD(-4000, SPEED_MIN, SPEED_2, SEC_HALF_V);					//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 3){
		driveD(-6000, SPEED_MIN, SPEED_3, SEC_HALF_V);						//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 4){
		driveD(-8000, SPEED_MIN, SPEED_4, SEC_HALF_V);						//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 5){
		driveD(-10000, SPEED_MIN, SPEED_5, SEC_HALF_V);						//半区画分指定減速度で減速走行。走行後は停止する
	}else if(run_mode == 6){
		driveD(-10000, SPEED_MIN, SPEED_6, SEC_HALF_V);						//半区画分指定減速度で減速走行。走行後は停止する
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionU
// 等速で半区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionU(void){
	full_led_write1(WHITE);
	control_start();
	driveU(SEC_HALF);													//半区画分等速走行。走行後は停止しない
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//half_sectionV
// 等速で斜め半区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionV(void){
	full_led_write1(WHITE);
	control_start();
	driveU(SEC_HALF_V);													//半区画分等速走行。走行後は停止しない
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_section
// 1区画分進んで停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_section(void){

	half_sectionA();													//半区画分加速走行
	half_sectionD();													//半区画分減速走行のち停止
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionA
// 1区画分加速する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionA(void){
	full_led_write1(BLUEGREEN);
	control_start();
	if(run_mode == 1){
		driveA(accel_hs, SPEED_1, speed_max_hs, SEC_HALF*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 2){
		driveA(accel_hs, SPEED_2, speed_max_hs, SEC_HALF*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 3){
		driveA(accel_hs, SPEED_3, speed_max_hs, SEC_HALF*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 4){
		driveA(accel_hs, SPEED_4, speed_max_hs, SEC_HALF*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 5){
		driveA(accel_hs, SPEED_5, speed_max_hs, SEC_HALF*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 6){
		driveA(accel_hs, SPEED_6, speed_max_hs, SEC_HALF*2);				//1区画分加速走行。走行後は停止しない
	}
	if(!MF.FLAG.SCND)get_wall_info();										//a壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionD
// 1区画分減速する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionD(void){
	full_led_write1(BLUE);
	control_start();
	if(run_mode == 1){
		driveD(-accel_hs, SPEED_1, speed_max_hs, SEC_HALF*2);			//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 2){
		driveD(-accel_hs, SPEED_2, speed_max_hs, SEC_HALF*2);			//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 3){
		driveD(-accel_hs, SPEED_3, speed_max_hs, SEC_HALF*2);			//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 4){
		driveD(-accel_hs, SPEED_4, speed_max_hs, SEC_HALF*2);			//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 5){
		driveD(-accel_hs, SPEED_5, speed_max_hs, SEC_HALF*2);			//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 6){
		driveD(-accel_hs, SPEED_6, speed_max_hs, SEC_HALF*2);			//1区画分減速走行。走行後は停止しない
	}
	if(!MF.FLAG.SCND)get_wall_info();									//a壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionVA
// 等速で斜め半区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionVA(void){
	full_led_write1(BLUEGREEN);
	control_start();
	if(run_mode == 1){
		driveA(accel_hs, SPEED_1, speed_max_hs, SEC_HALF_V*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 2){
		driveA(accel_hs, SPEED_2, speed_max_hs, SEC_HALF_V*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 3){
		driveA(accel_hs, SPEED_3, speed_max_hs, SEC_HALF_V*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 4){
		driveA(accel_hs, SPEED_4, speed_max_hs, SEC_HALF_V*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 5){
		driveA(accel_hs, SPEED_5, speed_max_hs, SEC_HALF_V*2);				//1区画分加速走行。走行後は停止しない
	}else if(run_mode == 6){
		driveA(accel_hs, SPEED_6, speed_max_hs, SEC_HALF_V*2);				//1区画分加速走行。走行後は停止しない
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionVD
// 等速で斜め半区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionVD(void){
	full_led_write1(BLUE);
	control_start();
	if(run_mode == 1){
		driveD(-accel_hs, SPEED_1, speed_max_hs, SEC_HALF_V*2);				//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 2){
		driveD(-accel_hs, SPEED_2, speed_max_hs, SEC_HALF_V*2);				//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 3){
		driveD(-accel_hs, SPEED_3, speed_max_hs, SEC_HALF_V*2);				//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 4){
		driveD(-accel_hs, SPEED_4, speed_max_hs, SEC_HALF_V*2);				//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 5){
		driveD(-accel_hs, SPEED_5, speed_max_hs, SEC_HALF_V*2);				//1区画分減速走行。走行後は停止しない
	}else if(run_mode == 6){
		driveD(-accel_hs, SPEED_6, speed_max_hs, SEC_HALF_V*2);				//1区画分減速走行。走行後は停止しない
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//one_sectionU
// 等速で1区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionU(void){
	full_led_write1(WHITE);
	control_start();
	driveU(SEC_HALF*2);													//1区画分等速走行。走行後は停止しない
	if(!MF.FLAG.SCND)get_wall_info();									//a壁情報を取得
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_R90
// 右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_R90(void){
	target_omega_z = 800;
	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = target_omega_z/180*M_PI * TREAD/2;
	speed_min_r = -1*target_omega_z/180*M_PI * TREAD/2;

	drive_start();														//a走行開始
	control_stop();
	while(degree_z > target_degree_z-80);

	accel_l = -10000;
	accel_r = 10000;
	speed_min_l = 100;
	speed_max_r = -100;

	while(degree_z > target_degree_z-90);

	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_R90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
	}else{
		turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
	}
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_L90
// 左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_L90(void){
	target_omega_z = 800;
	accel_l = -3000;
	accel_r = 3000;
	speed_min_l = -1*target_omega_z/180*M_PI * TREAD/2;
	speed_max_r = target_omega_z/180*M_PI * TREAD/2;

	drive_start();														//a走行開始
	control_stop();
	while(degree_z < target_degree_z+80);

	accel_l = 10000;
	accel_r = -10000;
	speed_max_l = -100;
	speed_min_r = 100;

	while(degree_z < target_degree_z+90);

	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_L90, 1);										//aマイクロマウス内部位置情報でも右回転処理&目標角度右90度
	}else{
		turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも右回転処理&目標角度右90度
	}
	drive_stop();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//rotate_180
// 180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_180(void){

	full_led_write1(GREEN);
	target_omega_z = 800;
	accel_l = 3000;
	accel_r = -3000;
	speed_max_l = target_omega_z/180*M_PI * TREAD/2;
	speed_min_r = -1*target_omega_z/180*M_PI * TREAD/2;

	drive_start();														//a走行開始
	control_stop();
	while(degree_z > target_degree_z-160);

	accel_l = -10000;
	accel_r = 10000;
	speed_min_l = 100;
	speed_max_r = -100;

	while(degree_z > target_degree_z-180);

	if(!MF.FLAG.XDIR){
		turn_dir(DIR_TURN_180, 1);										//aマイクロマウス内部位置情報でも180度回転処理&目標角度左180度
	}else{
		turn_dir(DIR_TURN_R180_8, 3);									//aマイクロマウス内部位置情報でも180度回転処理&目標角度左180度
	}
	drive_stop();

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_R90
// スラロームで左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_R90(void){
	full_led_write1(PURPLE);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, SLALOM_OFFSET_F, SLALOM_WALL_FL, SLALOM_WALL_FR);
		slalomR(-SLALOM_DEGACCEL, -SLALOM_OMEGA, -90, SPEED_1);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}
		slalomB(10000, SPEED_1, SLALOM_OFFSET_B);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, SLALOM_2_OFFSET_F, SLALOM_2_WALL_FL, SLALOM_2_WALL_FR);
		slalomR(-SLALOM_2_DEGACCEL, -SLALOM_2_OMEGA, -90, SPEED_2);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}
		slalomB(10000, SPEED_2, SLALOM_2_OFFSET_B-10);					//B-10
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, SLALOM_3_OFFSET_F, SLALOM_3_WALL_FL, SLALOM_3_WALL_FR);
		slalomR(-SLALOM_3_DEGACCEL, -SLALOM_3_OMEGA, -80, SPEED_3);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}
		slalomB(10000, SPEED_3, SLALOM_3_OFFSET_B);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, SLALOM_4_OFFSET_F, SLALOM_4_WALL_FL, SLALOM_4_WALL_FR);
		slalomR(-SLALOM_4_DEGACCEL, -SLALOM_4_OMEGA, -60, SPEED_4);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}
		slalomB(10000, SPEED_4, SLALOM_4_OFFSET_B);
	}else if(run_mode == 5){
		MF2.FLAG.TEMP = 1;
		slalomF(10000, SPEED_5, SLALOM_5_OFFSET_F, SLALOM_5_WALL_FL, SLALOM_5_WALL_FR);
		slalomR(-SLALOM_5_DEGACCEL, -SLALOM_5_OMEGA, -20, SPEED_5);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}
		slalomB(10000, SPEED_5, SLALOM_5_OFFSET_B);
		MF2.FLAG.TEMP = 0;
	}else if(run_mode == 6){
		MF2.FLAG.TEMP = 1;
		slalomF(10000, SPEED_5, SLALOM_5_OFFSET_F, SLALOM_5_WALL_FL, SLALOM_5_WALL_FR);
		MF2.FLAG.TEMP = 0;
		slalomR(-SLALOM_5_DEGACCEL, -SLALOM_5_OMEGA, -20, SPEED_5);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}
		MF2.FLAG.TEMP = 1;
		slalomB(10000, SPEED_5, SLALOM_5_OFFSET_B);
		MF2.FLAG.TEMP = 0;
	}
	if(!MF.FLAG.SCND)get_wall_info();									//a壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_L90
// スラロームで右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_L90(void){
	full_led_write1(YELLOW);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, SLALOM_OFFSET_F, SLALOM_WALL_FL, SLALOM_WALL_FR);
		slalomR(SLALOM_DEGACCEL, SLALOM_OMEGA, 90, SPEED_1);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_1, SLALOM_OFFSET_B);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, SLALOM_2_OFFSET_F, SLALOM_2_WALL_FL, SLALOM_2_WALL_FR);
		slalomR(SLALOM_2_DEGACCEL, SLALOM_2_OMEGA, 90, SPEED_2);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度左90度
		}
		slalomB(10000, SPEED_2, SLALOM_2_OFFSET_B);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, SLALOM_3_OFFSET_F, SLALOM_3_WALL_FL, SLALOM_3_WALL_FR);
		slalomR(SLALOM_3_DEGACCEL, SLALOM_3_OMEGA, 80, SPEED_3);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_3, SLALOM_3_OFFSET_B);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, SLALOM_4_OFFSET_F, SLALOM_4_WALL_FL, SLALOM_4_WALL_FR);
		slalomR(SLALOM_4_DEGACCEL, SLALOM_4_OMEGA, 60, SPEED_4);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_4, SLALOM_4_OFFSET_B);
	}else if(run_mode == 5){
		MF2.FLAG.TEMP = 1;
		slalomF(10000, SPEED_5, SLALOM_5_OFFSET_F, SLALOM_5_WALL_FL, SLALOM_5_WALL_FR);
		slalomR(SLALOM_5_DEGACCEL, SLALOM_5_OMEGA, 20, SPEED_5);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_5, SLALOM_5_OFFSET_B);
		MF2.FLAG.TEMP = 0;
	}else if(run_mode == 6){
		MF2.FLAG.TEMP = 1;
		slalomF(10000, SPEED_5, SLALOM_5_OFFSET_F, SLALOM_5_WALL_FL, SLALOM_5_WALL_FR);
		MF2.FLAG.TEMP = 0;
		slalomR(SLALOM_5_DEGACCEL, SLALOM_5_OMEGA, 20, SPEED_5);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		MF2.FLAG.TEMP = 1;
		slalomB(10000, SPEED_5, SLALOM_5_OFFSET_B);
		MF2.FLAG.TEMP = 0;
	}
	if(!MF.FLAG.SCND)get_wall_info();									//a壁情報を取得，片壁制御の有効・無効の判断
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_R90
// スラロームで右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_R90(void){
	full_led_write1(PURPLE);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, LSLALOM_OFFSET_F, LSLALOM_WALL_FL, LSLALOM_WALL_FR);
		slalomR(-LSLALOM_DEGACCEL, -LSLALOM_OMEGA, -90, SPEED_1);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_1, LSLALOM_OFFSET_B);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, LSLALOM_2_OFFSET_F, LSLALOM_2_WALL_FL, LSLALOM_2_WALL_FR);
		slalomR(-LSLALOM_2_DEGACCEL, -LSLALOM_2_OMEGA, -90, SPEED_2);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_2, LSLALOM_2_OFFSET_B);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, LSLALOM_3_OFFSET_F, LSLALOM_3_WALL_FL, LSLALOM_3_WALL_FR);
		slalomR(-LSLALOM_3_DEGACCEL, -LSLALOM_3_OMEGA, -90, SPEED_3);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_3, LSLALOM_3_OFFSET_B);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, LSLALOM_4_OFFSET_F, LSLALOM_4_WALL_FL, LSLALOM_4_WALL_FR);
		slalomR(-LSLALOM_4_DEGACCEL, -LSLALOM_4_OMEGA, -85, SPEED_4);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_4, LSLALOM_4_OFFSET_B);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, LSLALOM_5_OFFSET_F, LSLALOM_5_WALL_FL, LSLALOM_5_WALL_FR);
		slalomR(-LSLALOM_5_DEGACCEL, -LSLALOM_5_OMEGA, -80, SPEED_5);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_5, LSLALOM_5_OFFSET_B);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_6, LSLALOM_6_OFFSET_F, LSLALOM_6_WALL_FL, LSLALOM_6_WALL_FR);
		slalomR(-LSLALOM_6_DEGACCEL, -LSLALOM_6_OMEGA, -70, SPEED_6);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_6, LSLALOM_6_OFFSET_B);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_L90
// 大回りスラロームで右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_L90(void){
	full_led_write1(YELLOW);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, LSLALOM_OFFSET_F, LSLALOM_WALL_FL, LSLALOM_WALL_FR);
		slalomR(LSLALOM_DEGACCEL, LSLALOM_OMEGA, 90, SPEED_1);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_1, LSLALOM_OFFSET_B);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, LSLALOM_2_OFFSET_F, LSLALOM_2_WALL_FL, LSLALOM_2_WALL_FR);
		slalomR(LSLALOM_2_DEGACCEL, LSLALOM_2_OMEGA, 90, SPEED_2);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_2, LSLALOM_2_OFFSET_B);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, LSLALOM_3_OFFSET_F, LSLALOM_3_WALL_FL, LSLALOM_3_WALL_FR);
		slalomR(LSLALOM_3_DEGACCEL, LSLALOM_3_OMEGA, 90, SPEED_3);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_3, LSLALOM_3_OFFSET_B);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, LSLALOM_4_OFFSET_F, LSLALOM_4_WALL_FL, LSLALOM_4_WALL_FR);
		slalomR(LSLALOM_4_DEGACCEL, LSLALOM_4_OMEGA, 85, SPEED_4);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_4, LSLALOM_4_OFFSET_B);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, LSLALOM_5_OFFSET_F, LSLALOM_5_WALL_FL, LSLALOM_5_WALL_FR);
		slalomR(LSLALOM_5_DEGACCEL, LSLALOM_5_OMEGA, 80, SPEED_5);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_5, LSLALOM_5_OFFSET_B);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_6, LSLALOM_6_OFFSET_F, LSLALOM_6_WALL_FL, LSLALOM_6_WALL_FR);
		slalomR(LSLALOM_6_DEGACCEL, LSLALOM_6_OMEGA, 70, SPEED_6);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);										//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_6, LSLALOM_6_OFFSET_B);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_R180
// スラロームで右に180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_R180(void){
	full_led_write1(GREEN);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, LROTATE_OFFSET_F, LROTATE_WALL_FL, LROTATE_WALL_FR);
		slalomR(-LROTATE_DEGACCEL, -LROTATE_OMEGA, -180, SPEED_1);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_1, LROTATE_OFFSET_B);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, LROTATE_2_OFFSET_F, LROTATE_2_WALL_FL, LROTATE_2_WALL_FR);
		slalomR(-LROTATE_2_DEGACCEL, -LROTATE_2_OMEGA, -180, SPEED_2);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_2, LROTATE_2_OFFSET_B);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, LROTATE_3_OFFSET_F, LROTATE_3_WALL_FL, LROTATE_3_WALL_FR);
		slalomR(-LROTATE_3_DEGACCEL, -LROTATE_3_OMEGA, -170, SPEED_3);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_3, LROTATE_3_OFFSET_B);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, LROTATE_4_OFFSET_F, LROTATE_4_WALL_FL, LROTATE_4_WALL_FR);
		slalomR(-LROTATE_4_DEGACCEL, -LROTATE_4_OMEGA, -170, SPEED_4);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_4, LROTATE_4_OFFSET_B);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, LROTATE_5_OFFSET_F, LROTATE_5_WALL_FL, LROTATE_5_WALL_FR);
		slalomR(-LROTATE_5_DEGACCEL, -LROTATE_5_OMEGA, -170, SPEED_5);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_5, LROTATE_5_OFFSET_B);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_6, LROTATE_6_OFFSET_F, LROTATE_6_WALL_FL, LROTATE_6_WALL_FR);
		slalomR(-LROTATE_6_DEGACCEL, -LROTATE_6_OMEGA, -160, SPEED_6);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_R90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_R180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_6, LROTATE_6_OFFSET_B);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//Lslalom_L180
// 大回りスラロームで右に180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void Lslalom_L180(void){
	full_led_write1(GREEN);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, LROTATE_OFFSET_F, LROTATE_WALL_FL, LROTATE_WALL_FR);
		slalomR(LROTATE_DEGACCEL, LROTATE_OMEGA, 180, SPEED_1);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_1, LROTATE_OFFSET_B);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, LROTATE_2_OFFSET_F, LROTATE_2_WALL_FL, LROTATE_2_WALL_FR);
		slalomR(LROTATE_2_DEGACCEL, LROTATE_2_OMEGA, 180, SPEED_2);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_2, LROTATE_2_OFFSET_B);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, LROTATE_3_OFFSET_F, LROTATE_3_WALL_FL, LROTATE_3_WALL_FR);
		slalomR(LROTATE_3_DEGACCEL, LROTATE_3_OMEGA, 170, SPEED_3);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_3, LROTATE_3_OFFSET_B);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, LROTATE_4_OFFSET_F, LROTATE_4_WALL_FL, LROTATE_4_WALL_FR);
		slalomR(LROTATE_4_DEGACCEL, LROTATE_4_OMEGA, 170, SPEED_4);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_4, LROTATE_4_OFFSET_B);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, LROTATE_5_OFFSET_F, LROTATE_5_WALL_FL, LROTATE_5_WALL_FR);
		slalomR(LROTATE_5_DEGACCEL, LROTATE_5_OMEGA, 170, SPEED_5);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_5, LROTATE_5_OFFSET_B);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_6, LROTATE_6_OFFSET_F, LROTATE_6_WALL_FL, LROTATE_6_WALL_FR);
		slalomR(LROTATE_6_DEGACCEL, LROTATE_6_OMEGA, 160, SPEED_6);

		if(!MF.FLAG.XDIR){
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
			turn_dir(DIR_TURN_L90, 1);									//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}else{
			turn_dir(DIR_TURN_L180_8, 3);								//aマイクロマウス内部位置情報でも左回転処理&目標角度右90度
		}
		slalomB(10000, SPEED_6, LROTATE_6_OFFSET_B);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R45
// 区画中心から右に45度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R45(void){
	full_led_write1(PURPLE);
	if(run_mode == 1){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_1, V45_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_1, V45_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V45_DEGACCEL, -V45_OMEGA, -45, SPEED_1);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_1, V45_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_1, V45_OFFSET_B);
		}
	}else if(run_mode == 2){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_2, V45_2_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_2, V45_2_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V45_2_DEGACCEL, -V45_2_OMEGA, -45, SPEED_2);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_2, V45_2_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_2, V45_2_OFFSET_B);
		}
	}else if(run_mode == 3){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_3, V45_3_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_3, V45_3_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V45_3_DEGACCEL, -V45_3_OMEGA, -45, SPEED_3);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_3, V45_3_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_3, V45_3_OFFSET_B);
		}
	}else if(run_mode == 4){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_4, V45_4_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_4, V45_4_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V45_4_DEGACCEL, -V45_4_OMEGA, -45, SPEED_4);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_4, V45_4_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_4, V45_4_OFFSET_B);
		}
	}else if(run_mode == 5){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_5, V45_5_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_5, V45_5_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V45_5_DEGACCEL, -V45_5_OMEGA, -45, SPEED_5);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_5, V45_5_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_5, V45_5_OFFSET_B);
		}
	}else if(run_mode == 6){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_5, V45_5_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_5, V45_5_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V45_5_DEGACCEL, -V45_5_OMEGA, -45, SPEED_5);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_5, V45_5_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_5, V45_5_OFFSET_B);
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L45
// 区画中心から左に45度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L45(void){
	full_led_write1(YELLOW);
	if(run_mode == 1){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_1, V45_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_1, V45_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V45_DEGACCEL, V45_OMEGA, 45, SPEED_1);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_1, V45_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_1, V45_OFFSET_B);
		}
	}else if(run_mode == 2){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_2, V45_2_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_2, V45_2_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V45_2_DEGACCEL, V45_2_OMEGA, 45, SPEED_2);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_2, V45_2_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_2, V45_2_OFFSET_B);
		}
	}else if(run_mode == 3){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_3, V45_3_OFFSET_F+5, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_3, V45_3_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V45_3_DEGACCEL, V45_3_OMEGA, 45, SPEED_3);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_3, V45_3_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_3, V45_3_OFFSET_B);
		}
	}else if(run_mode == 4){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_4, V45_4_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_4, V45_4_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V45_4_DEGACCEL, V45_4_OMEGA, 45, SPEED_4);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_4, V45_4_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_4, V45_4_OFFSET_B);
		}
	}else if(run_mode == 5){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_5, V45_5_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_5, V45_5_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V45_5_DEGACCEL, V45_5_OMEGA, 45, SPEED_5);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_5, V45_5_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_5, V45_5_OFFSET_B);
		}
	}else if(run_mode == 6){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_5, V45_5_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_5, V45_5_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V45_5_DEGACCEL, V45_5_OMEGA, 45, SPEED_5);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_5, V45_5_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_5, V45_5_OFFSET_B);
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R45D
//a区画中心から右に45度回転する　ゴール用
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R45D(void){
	full_led_write1(PURPLE);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, V45_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V45_DEGACCEL, -V45_OMEGA, -45, SPEED_1);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_1, V45_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_1, 30);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, V45_2_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V45_2_DEGACCEL, -V45_2_OMEGA, -45, SPEED_2);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_2, V45_2_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_2, 30);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, V45_3_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V45_3_DEGACCEL, -V45_3_OMEGA, -45, SPEED_3);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_3, V45_3_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_3, 30);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, V45_4_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V45_4_DEGACCEL, -V45_4_OMEGA, -45, SPEED_4);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_4, V45_4_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_4, 30);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, V45_5_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V45_5_DEGACCEL, -V45_5_OMEGA, -45, SPEED_5);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_5, V45_5_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_5, 30);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_5, V45_5_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V45_5_DEGACCEL, -V45_5_OMEGA, -45, SPEED_5);

		turn_dir(DIR_TURN_R45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_5, V45_5_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_5, 30);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L45D
//a区画中心から左に45度回転する　ゴール用
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L45D(void){
	full_led_write1(YELLOW);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, V45_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V45_DEGACCEL, V45_OMEGA, 55, SPEED_1);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_1, V45_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_1, 30);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, V45_2_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V45_2_DEGACCEL, V45_2_OMEGA, 45, SPEED_2);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_2, V45_2_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_2, 30);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, V45_3_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V45_3_DEGACCEL, V45_3_OMEGA, 45, SPEED_3);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_3, V45_3_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_3, 30);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, V45_4_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V45_4_DEGACCEL, V45_4_OMEGA, 45, SPEED_4);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_4, V45_4_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_4, 30);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, V45_5_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V45_5_DEGACCEL, V45_5_OMEGA, 45, SPEED_5);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_5, V45_5_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_5, 30);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_5, V45_5_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V45_5_DEGACCEL, V45_5_OMEGA, 45, SPEED_5);

		turn_dir(DIR_TURN_L45_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_5, V45_5_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_5, 30);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R90
// 柱中心から右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R90(void){
	full_led_write1(YELLOW);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, V90_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(-V90_DEGACCEL, -V90_OMEGA, -90, SPEED_1);

		turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_1, V90_OFFSET_B);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, V90_2_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(-V90_2_DEGACCEL, -V90_2_OMEGA, -90, SPEED_2);

		turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_2, V90_2_OFFSET_B);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, V90_3_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(-V90_3_DEGACCEL, -V90_3_OMEGA, -90, SPEED_3);

		turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_3, V90_3_OFFSET_B);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, V90_4_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(-V90_4_DEGACCEL, -V90_4_OMEGA, -90, SPEED_4);

		turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_4, V90_4_OFFSET_B);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, V90_5_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(-V90_5_DEGACCEL, -V90_5_OMEGA, -90, SPEED_5);

		turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_5, V90_5_OFFSET_B);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_5, V90_5_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(-V90_5_DEGACCEL, -V90_5_OMEGA, -90, SPEED_5);

		turn_dir(DIR_TURN_R90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_5, V90_5_OFFSET_B);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L90
// 柱中心から左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L90(void){
	full_led_write1(PURPLE);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, V90_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(V90_DEGACCEL, V90_OMEGA, 90, SPEED_1);

		turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_1, V90_OFFSET_B);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, V90_2_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(V90_2_DEGACCEL, V90_2_OMEGA, 90, SPEED_2);

		turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_2, V90_2_OFFSET_B);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, V90_3_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(V90_3_DEGACCEL, V90_3_OMEGA, 90, SPEED_3);

		turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_3, V90_3_OFFSET_B);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, V90_4_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(V90_4_DEGACCEL, V90_4_OMEGA, 90, SPEED_4);

		turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_4, V90_4_OFFSET_B);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, V90_5_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(V90_5_DEGACCEL, V90_5_OMEGA, 90, SPEED_5);

		turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_5, V90_5_OFFSET_B);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_5, V90_5_OFFSET_F, NO_WALL, NO_WALL);
		slalomR(V90_5_DEGACCEL, V90_5_OMEGA, 90, SPEED_5);

		turn_dir(DIR_TURN_L90_8, 3);									//aマイクロマウス内部位置情報でも左回転処理

		slalomB(10000, SPEED_5, V90_5_OFFSET_B);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R135
// 区画中心から右に135度回転する　ゴール用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R135(void){
	full_led_write1(YELLOW);
	if(run_mode == 1){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_1, V135_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_1, V135_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V135_DEGACCEL, -V135_OMEGA, -135, SPEED_1);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_1, V135_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_1, V135_OFFSET_B);
		}
	}else if(run_mode == 2){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_2, V135_2_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_2, V135_2_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V135_2_DEGACCEL, -V135_2_OMEGA, -135, SPEED_2);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_2, V135_2_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_2, V135_2_OFFSET_B);
		}
	}else if(run_mode == 3){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_3, V135_3_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_3, V135_3_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V135_3_DEGACCEL, -V135_3_OMEGA, -135, SPEED_3);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_3, V135_3_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_3, V135_3_OFFSET_B);
		}
	}else if(run_mode == 4){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_4, V135_4_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_4, V135_4_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V135_4_DEGACCEL, -V135_4_OMEGA, -135, SPEED_4);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_4, V135_4_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_4, V135_4_OFFSET_B);
		}
	}else if(run_mode == 5){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_5, V135_5_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_5, V135_5_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V135_5_DEGACCEL, -V135_5_OMEGA, -135, SPEED_5);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_5, V135_5_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_5, V135_5_OFFSET_B);
		}
	}else if(run_mode == 6){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_5, V135_5_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_5, V135_5_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(-V135_5_DEGACCEL, -V135_5_OMEGA, -135, SPEED_5);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_5, V135_5_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_5, V135_5_OFFSET_B);
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L135
// 区画中心から左に135度回転する　ゴール用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L135(void){
	full_led_write1(PURPLE);
	if(run_mode == 1){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_1, V135_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_1, V135_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V135_DEGACCEL, V135_OMEGA, 135, SPEED_1);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_1, V135_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_1, V135_OFFSET_B);
		}
	}else if(run_mode == 2){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_2, V135_2_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_2, V135_2_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V135_2_DEGACCEL, V135_2_OMEGA, 135, SPEED_2);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_2, V135_2_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_2, V135_2_OFFSET_B);
		}
	}else if(run_mode == 3){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_3, V135_3_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_3, V135_3_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V135_3_DEGACCEL, V135_3_OMEGA, 135, SPEED_3);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_3, V135_3_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_3, V135_3_OFFSET_B);
		}
	}else if(run_mode == 4){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_4, V135_4_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_4, V135_4_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V135_4_DEGACCEL, V135_4_OMEGA, 135, SPEED_4);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_4, V135_4_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_4, V135_4_OFFSET_B);
		}
	}else if(run_mode == 5){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_5, V135_5_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_5, V135_5_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V135_5_DEGACCEL, V135_5_OMEGA, 135, SPEED_5);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_5, V135_5_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_5, V135_5_OFFSET_B);
		}
	}else if(run_mode == 6){
		if(!MF2.FLAG.V){
			slalomF(10000, SPEED_5, V135_5_OFFSET_F, NO_WALL, NO_WALL);
		}else{
			slalomF(10000, SPEED_5, V135_5_OFFSET_VF, NO_WALL, NO_WALL);
		}
		slalomR(V135_5_DEGACCEL, V135_5_OMEGA, 135, SPEED_5);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		if(!MF2.FLAG.V){
			slalomB(10000, SPEED_5, V135_5_OFFSET_VB);
		}else{
			slalomB(10000, SPEED_5, V135_5_OFFSET_B);
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_R135D
// 区画中心から右に135度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_R135D(void){
	full_led_write1(YELLOW);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, V135_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V135_DEGACCEL, -V135_OMEGA, -135, SPEED_1);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_1, V135_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_1, 30);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, V135_2_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V135_2_DEGACCEL, -V135_2_OMEGA, -135, SPEED_2);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_2, V135_2_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_2, 30);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, V135_3_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V135_3_DEGACCEL, -V135_3_OMEGA, -135, SPEED_3);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_3, V135_3_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_3, 30);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, V135_4_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V135_4_DEGACCEL, -V135_4_OMEGA, -135, SPEED_3);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_4, V135_4_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_4, 30);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, V135_5_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V135_5_DEGACCEL, -V135_5_OMEGA, -135, SPEED_5);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_5, V135_5_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_5, 30);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_5, V135_5_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(-V135_5_DEGACCEL, -V135_5_OMEGA, -135, SPEED_5);

		turn_dir(DIR_TURN_R135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_5, V135_5_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_5, 30);
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_L135D
// 区画中心から左に135度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_L135D(void){
	full_led_write1(PURPLE);
	if(run_mode == 1){
		slalomF(10000, SPEED_1, V135_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V135_DEGACCEL, V135_OMEGA, 135, SPEED_1);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_1, V135_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_1, 30);
	}else if(run_mode == 2){
		slalomF(10000, SPEED_2, V135_2_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V135_2_DEGACCEL, V135_2_OMEGA, 135, SPEED_2);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_2, V135_2_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_2, 30);
	}else if(run_mode == 3){
		slalomF(10000, SPEED_3, V135_3_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V135_3_DEGACCEL, V135_3_OMEGA, 135, SPEED_3);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_3, V135_3_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_3, 30);
	}else if(run_mode == 4){
		slalomF(10000, SPEED_4, V135_4_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V135_4_DEGACCEL, V135_4_OMEGA, 135, SPEED_4);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_4, V135_4_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_3, 30);
	}else if(run_mode == 5){
		slalomF(10000, SPEED_5, V135_5_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V135_5_DEGACCEL, V135_5_OMEGA, 135, SPEED_5);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_5, V135_5_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_5, 30);
	}else if(run_mode == 6){
		slalomF(10000, SPEED_5, V135_5_OFFSET_VF, NO_WALL, NO_WALL);
		slalomR(V135_5_DEGACCEL, V135_5_OMEGA, 135, SPEED_5);

		turn_dir(DIR_TURN_L135_8, 3);									//aマイクロマウス内部位置情報でも左回転処理
		MF2.FLAG.V = (MF2.FLAG.V+1)%2;

		slalomB(10000, SPEED_5, V135_5_OFFSET_VB-30);
		driveD(-30000, SPEED_MIN, SPEED_5, 30);
	}
}


/*----------------------------------------------------------
		テスト関数
----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//test_select
// a走行系テスト選択
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void test_select(void){
	int mode = 0;
	printf("Test Select, Mode : %d\n", mode);

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
			  switch(mode){
				case 1:
					init_test();
					break;

				case 2:
					slalom_test();
					break;

				case 3:
					v_test();
					break;

				case 4:
					pass_test();

				case 5:
					goal_test();

				case 6:
					sample_course_run();

			  }
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//init_test
// a初期基幹関数走行テスト
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void init_test(void){

	int mode = 0;
	printf("Test Init Run, Mode : %d\n", mode);

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
			  get_base();

			  switch(mode){
				case 0:
					get_base();
					break;
				case 1:
					//----4区画等速走行----
					printf("4 Section, Forward, Constant Speed.\n");
					for(int i = 0; i < 1; i++){
						driveC(SEC_HALF*2);			//a一区画のパルス分デフォルトインターバルで走行
					}
					break;
				case 2:
					//----right90度回転----
					printf("Rotate R90.\n");
					for(int i = 0; i < 8; i++){
						rotate_R90();				//16回右90度回転、つまり4周回転
					}
					break;
				case 3:
					//----left90度回転----
					printf("Rotate L90.\n");
					for(int i = 0; i < 8; i++){
						rotate_L90();				//16回左90度回転、つまり4周回転
					}
					break;
				case 4:
					//----180度回転----
					printf("Rotate 180.\n");
					for(int i = 0; i < 4; i++){
						rotate_180();				//8回右180度回転、つまり4周回転
					}
					break;
				case 5:
					//----4区画連続走行----
					printf("4 Section, Forward, Continuous.\n");
					get_base();
					run_mode = 2;
					half_sectionA();				//a半区画のパルス分加速しながら走行
					for(int i = 0; i < 5-1; i++){
						one_sectionU();				//a一区画のパルス分等速走行
					}
					half_sectionD();				//a半区画のパルス分減速しながら走行。走行後は停止する
					break;
				case 6:
					//----a宴会芸モード----
					target_degree_z = degree_z;
					accel_l = 5000;

					MF2.FLAG.ENKAI = 1;

					while(1);
					break;
				case 7:
					//----a停止時前壁補正モード----
					MF.FLAG.FWALL = 1;
					MF.FLAG.DRV = 1;
					while(1);
					break;
			  }
			  dist_l = 0;
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_test
// aスラローム走行テスト
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_test(void){

	int mode = 0;
	printf("Test Slalom Run, Mode : %d\n", mode);

	run_select();
	dist_l = 0;

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode < 8){
				  full_led_write1(WHITE);
			  }else if(mode < 16){
				  full_led_write1(BLUEGREEN);
			  }else{
				  full_led_write1(PURPLE);
			  }
			  if(mode > 23){
				  mode = 0;
			  }
			  printf("Mode : %d\n", mode);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 8){
				  full_led_write1(WHITE);
			  }else if(mode < 16){
				  full_led_write1(BLUEGREEN);
			  }else{
				  full_led_write1(PURPLE);
			  }
			  if(mode < 0){
				  mode = 23;
			  }
			  printf("Mode : %d\n", mode);
		  }
		  if(dist_l <= -20){
			  dist_l = 0;
			  drive_ready();
			  get_base();

			  switch(mode){
				case 0:
					get_base();
					break;
				case 1:
					//----slalom右折----
					printf("slalom turn right .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						slalom_R90();				//1回右90度回転、つまり1/4周回転
					}
					half_sectionD();
					break;
				case 2:
					//----slalom左折----
					printf("slalom turn left .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						slalom_L90();				//1回左90度回転、つまり1/4周回転
					}
					half_sectionD();
					break;
				case 3:
					//----Lslalom右折----
					printf("Lslalom turn right .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						Lslalom_R90();				//1回右90度回転、つまり1/4周回転
					}
					half_sectionD();
					break;
				case 4:
					//----Lslalom左折----
					printf("Lslalom turn left .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						Lslalom_L90();				//1回左90度回転、つまり1/4周回転
					}
					half_sectionD();
					break;
				case 5:
					//----Lslalom右180----
					printf("Lslalom turn right & right .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						Lslalom_R180();				//1回右180度回転、つまり1/2周回転
					}
					half_sectionD();
					break;
				case 6:
					//----Lslalom左180----
					printf("Lslalom turn left & left .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						Lslalom_L180();				//1回左180度回転、つまり1/2周回転
					}
					half_sectionD();
					break;
				case 7:
					break;
				case 8:
					break;
				case 9:
					//----slalom右折----
					printf("slalom turn right .\n");
					half_sectionA();
					for(int i = 0; i < 8; i++){
						slalom_R90();				//8回右90度回転、つまり2周回転
						one_sectionU();
					}
					half_sectionD();
					break;
				case 10:
					//----slalom左折----
					printf("slalom turn left .\n");
					half_sectionA();
					MF2.FLAG.LOG = 1;
					for(int i = 0; i < 8; i++){
						slalom_L90();				//8回左90度回転、つまり2周回転
						one_sectionU();
					}
					MF2.FLAG.LOG = 0;
					half_sectionD();
/*					while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
					printf("omega start\n");
					for(int j = 0; j < log_allay; j++){
						printf("%d\n", get_omega[j]);
					}
					printf("omega end\n");
					while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
					for(int j = 0; j < log_allay; j++){
						printf("%d\n", get_speed_l[j]);
					}
					printf("l end\n");
					while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET);
					printf("r start\n");
					for(int j = 0; j < log_allay; j++){
						printf("%d\n", get_speed_r[j]);
					}
					printf("r end\n");
*/					break;
				case 11:
					//----Lslalom右折----
					printf("Lslalom turn right .\n");
					half_sectionA();
					for(int i = 0; i < 8; i++){
						Lslalom_R90();				//8回右90度回転、つまり2周回転
					}
					half_sectionD();
					break;
				case 12:
					//----Lslalom左折----
					printf("Lslalom turn left .\n");
					half_sectionA();
					for(int i = 0; i < 8; i++){
						Lslalom_L90();				//8回左90度回転、つまり2周回転
					}
					half_sectionD();
					break;
				case 13:
					//----Lslalom右180----
					printf("Lslalom turn right & right .\n");
					half_sectionA();
					for(int i = 0; i < 8; i++){
						Lslalom_R180();				//8回右180度回転、つまり4周回転
					}
					half_sectionD();
					break;
				case 14:
					//----Lslalom左180----
					printf("Lslalom turn left & left .\n");
					half_sectionA();
					for(int i = 0; i < 8; i++){
						Lslalom_L180();				//8回左180度回転、つまり4周回転
					}
					half_sectionD();
					break;
				case 15:
					break;
				case 16:
					break;

			  }
			  drive_break();
			  dist_l = 0;
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//v_test
// a斜め走行テスト
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void v_test(void){

	int mode = 0;
	printf("Test V Run, Mode : %d\n", mode);

	run_select();

	while(1){
		led_write(mode & 0b001, mode & 0b010, mode & 0b100);
		  if(dist_r >= 20){
			  mode++;
			  dist_r = 0;
			  if(mode > 23){
				  mode = 0;
			  }
			  if(mode < 8){
				  full_led_write1(WHITE);
			  }else if(mode < 16){
				  full_led_write1(BLUEGREEN);
			  }else{
				  full_led_write1(PURPLE);
			  }
			  printf("Mode : %d\n", mode);
		  }
		  if(dist_r <= -20){
			  mode--;
			  dist_r = 0;
			  if(mode < 0){
				  mode = 23;
			  }
			  if(mode < 8){
				  full_led_write1(WHITE);
			  }else if(mode < 16){
				  full_led_write1(BLUEGREEN);
			  }else{
				  full_led_write1(PURPLE);
			  }
			  printf("Mode : %d\n", mode);
		  }
		  if(dist_l <= -20){
			  dist_l = 0;
			  drive_ready();

			  MF.FLAG.XDIR = 1;
			  MF2.FLAG.V = 0;
			  get_base();

			  switch(mode){
				case 0:
					get_base();
					break;
				case 1:
					//----V右45----
					printf("V 45 right .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						v_R45();
					}
					half_sectionVD();
					break;
				case 2:
					//----V左45----
					printf("V 45 left .\n");
					half_sectionA();
					for(int i = 0; i < 1; i++){
						v_L45();
					}
					half_sectionVD();
					break;
				case 3:
					//----V右90----
					printf("V 90 right .\n");
					half_sectionVA();
//					v_R45();
					for(int i = 0; i < 1; i++){
						v_R90();
					}
//					v_R45();
					half_sectionVD();
					break;
				case 4:
					//----V左90----
					printf("V 90 left .\n");
					half_sectionVA();
//					v_L45();
					for(int i = 0; i < 1; i++){
						v_L90();
					}
//					v_L45();
					half_sectionVD();
					break;
				case 5:
					//----V右135----
					printf("V 135 right .\n");
					half_sectionA();
//					v_R45();
					for(int i = 0; i < 1; i++){
						v_R135();
					}
//					v_R45();
					half_sectionVD();
					break;
				case 6:
					//----V左135----
					printf("V 135 left .\n");
					half_sectionA();
//					v_L45();
					for(int i = 0; i < 1; i++){
						v_L135();
					}
//					v_L45();
					half_sectionVD();
					break;
				case 7:
					break;
				case 8:
					break;
				case 9:
					//----V右45----
					printf("V 45 right .\n");
					half_sectionA();
					for(int i = 0; i < 8; i++){
						v_R45();
					}
					half_sectionD();
					break;
				case 10:
					//----V左45----
					printf("V 45 left .\n");
					half_sectionA();
					for(int i = 0; i < 8; i++){
						v_L45();
					}
					half_sectionD();
					break;
				case 11:
					//----V右90----
					printf("V 90 right .\n");
					half_sectionVA();
//					v_R45();
					for(int i = 0; i < 8; i++){
						v_R90();
					}
//					v_R45();
					half_sectionVD();
					break;
				case 12:
					//----V左90----
					printf("V 90 left .\n");
					half_sectionVA();
//					v_L45();
					for(int i = 0; i < 8; i++){
						v_L90();
					}
//					v_L45();
					half_sectionVD();
					break;
				case 13:
					//----V右135----
					printf("V 135 right .\n");
					half_sectionA();
//					v_R45();
					for(int i = 0; i < 4; i++){
						v_R135();
					}
//					v_R45();
					half_sectionD();
					break;
				case 14:
					//----V左135----
					printf("V 135 left .\n");
					half_sectionA();
//					v_L45();
					for(int i = 0; i < 4; i++){
						v_L135();
					}
//					v_L45();
					half_sectionD();
					break;
				case 15:
					break;
				case 16:
					break;
				case 17:
					//----V右45D----
					printf("V 45 right .\n");
					MF2.FLAG.V = 1;
					half_sectionVA();
					for(int i = 0; i < 1; i++){
//						v_R45D();
						v_R45();
					}
					half_sectionD();
					break;
				case 18:
					//----V左45D----
					printf("V 45 left .\n");
					MF2.FLAG.V = 1;
					half_sectionVA();
					for(int i = 0; i < 1; i++){
//						v_L45D();
						v_L45();
					}
					half_sectionD();
					break;
				case 19:
					//----V右135D----
					printf("V 135 right .\n");
					MF2.FLAG.V = 1;
					half_sectionVA();
					for(int i = 0; i < 1; i++){
//						v_R135D();
						v_R135();
					}
					half_sectionD();
					break;
				case 20:
					//----V左135D----
					printf("V 135 left .\n");
					MF2.FLAG.V = 1;
					half_sectionVA();
					for(int i = 0; i < 1; i++){
//						v_L135D();
						v_L135();
					}
					half_sectionD();
					break;
			  }
			  drive_break();
			  dist_l = 0;
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//pass_test
// pass圧縮走行テスト
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void pass_test(void){

	int mode = 0;
	printf("Test pass Run, Mode : %d\n", mode);

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
			  MF2.FLAG.V = 0;

			  switch(mode){
				case 0:
					//----a一次探索スラローム走行----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 4000;
					speed_max_hs = 1000;
					start_mode = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 1:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース Middle Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 2:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1600;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 3:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High High Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 4;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 2000;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 4:
					//----a直線と大回り圧縮と斜め Middle Speedｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 4;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 5:
					//----a直線と大回り圧縮と斜め High Speedｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1600;

					pass_mode = 4;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 7:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High Speed----
					printf("pass press 3-4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 5;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 2000;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
			  }
			  dist_l = 0;
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//goal_test
// a複数マスgoal走行テスト
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void goal_test(void){

	int mode = 0;
	printf("Test goal Run, Mode : %d\n", mode);

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
			  MF2.FLAG.V = 0;

			  switch(mode){
				case 0:
					//----a一次探索スラローム走行----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 1;
					accel_hs = 5000;
					speed_max_hs = 800;
					start_mode = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 1:
					//----a一次探索スラローム走行 4マスgoal----
					printf("First Run. (4 goal)\n");

					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 800;
					start_mode = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();
					while(dist_l < 30);
					printf("x: %d, y:%d, dir:%d\n", mouse.x, mouse.y, mouse.dir);

					break;
				case 2:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース Middle Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 1;
					accel_hs = 5000;
					speed_max_hs = 1200;

//					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 3:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース Middle Speed 4マスgoal----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

//					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 4:
					//----a直線と大回り圧縮と斜め Middle Speedｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 1;
					accel_hs = 5000;
					speed_max_hs = 1200;

//					pass_mode = 4;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();
					break;
				case 5:
					//----a直線と大回り圧縮と斜め Middle Speed 4マスgoalｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

//					pass_mode = 4;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 6:
					//----a一次探索スラローム走行　重ね探索----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					MF.FLAG.SRC2 = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 1;
					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
				case 7:
					//----a一次探索スラローム走行　重ね探索 4マスgoal----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					MF.FLAG.SRC2 = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC2();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
			  }
			  dist_l = 0;
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//sample_course_run
//a試験走行モード
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void sample_course_run(void){

	int mode = 0;
	printf("Sample Course Run, Mode : %d\n", mode);

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
					//----aサンプルコース1　爆速ターン連続----
					run_mode = 5;

					half_sectionA();
					slalom_R90();
					slalom_L90();
					slalom_L90();
					slalom_R90();
					slalom_R90();
					for(int k=0; k<5; k++){
						slalom_R90();
						slalom_R90();
						slalom_L90();
						slalom_L90();
						slalom_R90();
						slalom_R90();
					}
					half_sectionD();
					break;

				case 2:
					//----aサンプルコース1　超信地----
					run_mode = 5;

					half_sectionA();
					slalom_R90();
					slalom_L90();
					slalom_R90();
					slalom_L90();
					slalom_R90();
					half_sectionD();
					break;

				case 3:
					//---aサンプルコース2　スラローム----
					half_sectionA();
					slalom_R90();
					slalom_R90();
					half_sectionD();
					break;

				case 4:
					break;

				case 5:
					//----aスラローム走行&全面探索スラローム走行----
					printf("Slalom Run & All Map Run.\n");

					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 800;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchE();

					searchC();
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 6:
					//----aスラローム走行&全面探索スラローム走行----
					printf("Slalom Run & All Map Run.\n");

					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 800;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchE();

					searchC();
					goal_x = 7;
					goal_y = 7;

					break;

				case 7:
					break;

			  }
			  dist_l = 0;
		  }
	}
}


/*----------------------------------------------------------
		走行モード選択関数
----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//simple_run
// a超信地走行モード
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void simple_run(void){

	int mode = 0;
	printf("Simple Run, Mode : %d\n", mode);

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
					break;
				case 1:
					//----a一次探索走行----
					printf("First Run.\n");

					MF.FLAG.SCND = 0;
					run_mode = 2;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchA();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchA();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 2:
					//----a一次探索連続走行----
					printf("First Run. (Continuous)\n");

					MF.FLAG.SCND = 0;
					run_mode = 2;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchB();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchB();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					break;

				case 3:
					//----a二次探索走行----
					printf("Second Run. (Continuous)\n");

					MF.FLAG.SCND = 1;
					run_mode = 2;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchB();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchB();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

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


//+++++++++++++++++++++++++++++++++++++++++++++++
//slalom_run
// aスラローム走行モード
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void slalom_run(void){

	int mode = 0;
	printf("Slalom Run, Mode : %d\n", mode);

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
					//----a一次探索スラローム走行----
					printf("First Run. (Slalom)\n");

					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 0;
					run_mode = 2;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 800;


					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 1:
					//----a二次探索スラローム走行----
					printf("Second Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					run_mode = 2;
					start_mode = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 2:
					//----a二次探索スラローム走行+既知区間加速----
					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					run_mode = 2;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 600;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 3:
					//----a二次探索スラローム走行+既知区間加速----
					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					run_mode = 2;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 1000;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 4:
					//----a二次探索スラローム走行+既知区間加速----
					printf("Second Run. (Slalom+accel)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					run_mode = 2;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 1500;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 5:
					//----a二次走行スラローム+直線優先----
					printf("High Speed Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					HAL_Delay(5000);
					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 6:
					//----a二次走行スラロームHigh Speed+直線優先+既知区間加速----
					printf("High Speed Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.STRAIGHT = 1;
					MF.FLAG.ACCL2 = 1;
					run_mode = 3;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					HAL_Delay(5000);
					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 7:
					//----a二次走行スラロームHigh Speed+直線優先+既知区間加速----
					printf("High Speed Run. (Slalom)\n");

					MF.FLAG.SCND = 1;
					MF.FLAG.STRAIGHT = 1;
					MF.FLAG.ACCL2 = 1;
					run_mode = 3;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 1600;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					HAL_Delay(5000);
					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

			  }
			  dist_l = 0;
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//perfect_run
// a本番用走行モード
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void perfect_run(void){

	int mode = 0;
	printf("Perfect Run, Mode : %d\n", mode);

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
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 4000;
					speed_max_hs = 1000;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 1:
					//----a一次重ね探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					MF.FLAG.SRC2 = 1;

					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 4000;
					speed_max_hs = 1000;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC2();
					start_mode = 1;
					goal_mode = 1;

					HAL_Delay(2000);

					rotate_180();											//180度回転
					MF.FLAG.SCND = 1;
					MF.FLAG.SRC2 = 0;
					goal_x = goal_y = 0;
					run_mode = 3;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 2:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1600;

//					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 3:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High High Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 4;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 2000;

//					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 4:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High High Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 4;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 15000;
					speed_max_hs = 2500;

//					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 5:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High Speed----
					printf("pass press 3-4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 5;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 25000;
					speed_max_hs = 3000;

//					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 6:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High Speed----
					printf("pass press 3-4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 5;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 2000;

//					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;

				case 7:
					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース High Speed----
					printf("pass press 3-4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 5;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 22000;
					speed_max_hs = 3000;

//					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = GOAL_X;
					goal_y = GOAL_Y;
					break;
			  }
			  dist_l = 0;
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//perfect_slalom
// a本番用スラローム走行モード
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void perfect_slalom(void){

	int mode = 0;
	printf("Perfect Slalom, Mode : %d\n", mode);

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
					break;

				case 1:
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = 7;
					goal_y = 7;
					break;

				case 2:
					//----a二次走行スラローム+既知区間加速走行 speed1----
					printf("First Run. (Continuous)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = 7;
					goal_y = 7;
					break;

				case 3:
					//----a二次探索スラロームHigh Speed----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 0;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();
					goal_x = 7;
					goal_y = 7;
					break;

				case 4:
					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = 7;
					goal_y = 7;
					break;

				case 5:
					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1600;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = 7;
					goal_y = 7;
					break;

				case 6:
					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 2000;
					goal_x = 7;
					goal_y = 7;

					get_base();

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					goal_x = 7;
					goal_y = 7;
					break;

				case 7:
					perfect_pass();
					break;
			  }
			  dist_l = 0;
		  }
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//perfect_pass
// a本番用pass圧縮走行モード
// a引数：なし
// a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void perfect_pass(void){

	int mode = 0;
	printf("Perfect Pass Press, Mode : %d\n", mode);

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
					break;

				case 1:
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 4000;
					speed_max_hs = 1000;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					goal_x = 7;
					goal_y = 7;
					break;

				case 2:
					//----a直線と大回り圧縮----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1000;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = 7;
					goal_y = 7;
					break;

				case 3:
					//----a直線と大回り圧縮と斜めｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1000;

					pass_mode = 4;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					goal_x = 7;
					goal_y = 7;
					break;

				case 4:
					//----a直線と大回り圧縮 High Speed----
					printf("pass press 3-2.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					goal_mode = 2;
					start_mode = 0;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					goal_x = 7;
					goal_y = 7;
					break;

				case 5:
					//----a直線と大回り圧縮と斜め High Speedｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 4;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					goal_x = 7;
					goal_y = 7;
					break;

				case 6:
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 4000;
					speed_max_hs = 1000;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					degree_z = target_degree_z;
					HAL_Delay(2000);

/*
					//----a二次探索スラローム+既知区間加速走行 speed2----
					printf("Second Run. (Continuous)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					degree_z = target_degree_z;
					HAL_Delay(2000);
*/

					//----a直線と大回り圧縮----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					start_mode = 0;

					HAL_Delay(2000);


					//----a直線と大回り圧縮と斜め----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 4;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					start_mode = 0;

					HAL_Delay(2000);

/*
					//----a二次探索スラロームHigh Speed + 既知区間加速----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 1200;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					degree_z = target_degree_z;
					HAL_Delay(2000);


					//----a二次探索スラロームHigh Speed + 既知区間加速 Speed2----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 1600;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					degree_z = target_degree_z;
					HAL_Delay(2000);


					//----a二次探索スラロームHigh Speed + 既知区間加速 Speed3----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 20000;
					speed_max_hs = 2000;
					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();
*/

					//----a直線と大回り圧縮 High Speed----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 6000;
					speed_max_hs = 1600;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					start_mode = 0;

					HAL_Delay(2000);


					//----a直線と大回り圧縮と斜めｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 6000;
					speed_max_hs = 1600;

					pass_mode = 4;

					goal_x = GOAL_X;
					goal_y = GOAL_Y;

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					break;

				case 7:
					//----a一次探索スラローム走行----
					printf("First Run.\n");
					MF.FLAG.SCND = 0;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 0;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 4000;
					speed_max_hs = 1000;

					goal_x = 7;
					goal_y = 7;

					get_base();

					searchC();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchC();

					degree_z = target_degree_z;
					HAL_Delay(2000);

/*
					//----a二次探索スラローム+既知区間加速走行 speed2----
					printf("Second Run. (Continuous)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;
					goal_x = 7;
					goal_y = 7;

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					degree_z = target_degree_z;
					HAL_Delay(2000);
*/

					//----a直線と大回り圧縮(adv_posを停止)+半区画ベース----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = 7;
					goal_y = 7;

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					start_mode = 0;
					HAL_Delay(2000);


					//----a直線と大回り圧縮と斜めｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 2;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 5000;
					speed_max_hs = 1200;

					pass_mode = 4;

					goal_x = 7;
					goal_y = 7;

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					start_mode = 0;

					HAL_Delay(2000);

					//----a二次探索スラロームHigh Speed + 既知区間加速----
/*					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 1600;
					goal_x = 7;
					goal_y = 7;

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					degree_z = target_degree_z;
					HAL_Delay(2000);


					//----a二次探索スラロームHigh Speed + 既知区間加速 Speed2----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 10000;
					speed_max_hs = 2000;

					goal_x = 7;
					goal_y = 7;

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();

					degree_z = target_degree_z;
					HAL_Delay(2000);


					//----a二次探索スラロームHigh Speed + 既知区間加速 Speed3----
					printf("Second Run. (Slalom)\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 20000;
					speed_max_hs = 2500;

					goal_x = 7;
					goal_y = 7;

					searchD();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchD();
*/

					//----a直線と大回り圧縮 High Speed----
					printf("pass press 3.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 6000;
					speed_max_hs = 1600;

					pass_mode = 3;						//a半区画ベースでroute配列生成

					goal_x = 7;
					goal_y = 7;

					searchF3();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF3();

					driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
					degree_z = target_degree_z;
					start_mode = 0;

					HAL_Delay(2000);


					//----a直線と大回り圧縮と斜め High Speedｰｰｰｰ
					printf("pass press 4.\n");
					MF.FLAG.SCND = 1;
					MF.FLAG.ACCL2 = 1;
					MF.FLAG.STRAIGHT = 1;
					run_mode = 3;
					start_mode = 0;
					goal_mode = 2;
					accel_hs = 6000;
					speed_max_hs = 1600;

					pass_mode = 4;

					goal_x = 7;
					goal_y = 7;

					searchF4();
					HAL_Delay(2000);

					goal_x = goal_y = 0;
					searchF4();

					break;
			  }
			  dist_l = 0;
		  }
	}
}
