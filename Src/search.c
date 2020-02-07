
#include "global.h"

void search_init(void){

	//----a探索系----
	goal_x = GOAL_X;        		//GOAL_Xはglobal.hにマクロ定義あり
	goal_y = GOAL_Y;        		//GOAL_Yはglobal.hにマクロ定義あり
	map_Init();						//aマップの初期化
	mouse.x = 0;
	mouse.y = 0;					//a現在地の初期化
	mouse.dir = 0;					//aマウスの向きの初期化
}


/*===========================================================
		探索系関数
===========================================================*/
/*-----------------------------------------------------------
		足立法探索走行A（1区画走行）
-----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//searchA
//a1区画走行でgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchA(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}

	//====aスタート位置壁情報取得====
	get_wall_info();										//a壁情報の初期化, 後壁はなくなる
	wall_info &= ~0x88;										//a前壁は存在するはずがないので削除する
	write_map();											//a壁情報を地図に記入

	//====a歩数マップ・経路作成====
	r_cnt = 0;												//a経路カウンタの初期化
	make_smap();											//a歩数マップ作成
	make_route();											//a最短経路探索（route配列に動作が格納される）

	//====a探索走行====
	do{
		//----a進行----
		switch(route[r_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//----a前進----
			case 0x88:
				break;
			//----a右折----
			case 0x44:
				rotate_R90();								//a右回転
				break;
			//----180回転----
			case 0x22:
				rotate_180();								//180度回転
				if(wall_info & 0x88){
					set_position();
				}
				break;
			//----a左折----
			case 0x11:
				rotate_L90();								//a左回転
				break;
		}

		drive_wait();
		one_section();										//a前進する
		drive_wait();

		adv_pos();											//aマイクロマウス内部位置情報でも前進処理
		conf_route();										//a最短経路で進行可能か判定

	}while((mouse.x != goal_x) || (mouse.y != goal_y));		//a現在座標とgoal座標が等しくなるまで実行

	printf("goal\n");
	HAL_Delay(500);										//aスタートでは***2秒以上***停止しなくてはならない
	rotate_180();											//180度回転

	if(!MF.FLAG.SCND){
		store_map_in_eeprom();
	}
}


/*-----------------------------------------------------------
		足立法探索走行B（連続走行）
-----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//searchB
//a連続走行でgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchB(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}

	//====aスタート位置壁情報取得====
	get_wall_info();										//a壁情報の初期化, 後壁はなくなる
	wall_info &= ~0x88;										//a前壁は存在するはずがないので削除する
	write_map();											//a壁情報を地図に記入

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();
	adv_pos();

	//====a歩数マップ・経路作成====
	write_map();											//a壁情報を地図に記入
	r_cnt = 0;												//a経路カウンタの初期化
	make_smap();											//a歩数マップ作成
	make_route();											//a最短経路探索（route配列に動作が格納される）

	//====a探索走行====
	do{
		//----a進行----
		switch(route[r_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//----a前進----
			case 0x88:
				one_sectionU();
				break;
			//----a右折----
			case 0x44:
				half_sectionD();
				rotate_R90();
				half_sectionA();
				break;
			//----180回転----
			case 0x22:
				half_sectionD();
				rotate_180();
				if(wall_info & 0x88){
					set_position();
				}
				half_sectionA();
				break;
			//----a左折----
			case 0x11:
				half_sectionD();
				rotate_L90();
				half_sectionA();
				break;
		}
		adv_pos();
		conf_route();

	}while((mouse.x != goal_x) || (mouse.y != goal_y));

	half_sectionD();

	HAL_Delay(500);
	rotate_180();											//180度回転

	if(!MF.FLAG.SCND){
		store_map_in_eeprom();
	}
}


/*-----------------------------------------------------------
		足立法探索走行C（スラローム走行）
-----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//searchC
//aスラローム走行でgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchC(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}

	//====aスタート位置壁情報取得====
	if(!MF.FLAG.SCND)get_wall_info();										//a壁情報の初期化, 後壁はなくなる
	if(!MF.FLAG.SCND)wall_info &= ~0x88;									//a前壁は存在するはずがないので削除する
	if(!MF.FLAG.SCND)write_map();											//a壁情報を地図に記入

	if(MF2.FLAG.GOAL){
//		HAL_Delay(500);
		rotate_180();											//180度回転
//		driveC2(SETPOS_BACK);         							//a尻を当てる程度に後退。回転後に停止する
//		degree_z = target_degree_z;
//		start_mode = 0;
		start_mode = 1;
		goal_mode = 1;
		HAL_Delay(500);
	}

	//====aスタート位置壁情報取得====
/*	if(!MF.FLAG.SCND || !MF2.FLAG.GOAL)get_wall_info();						//a壁情報の初期化, 後壁はなくなる
	if(!MF.FLAG.SCND || !MF2.FLAG.GOAL)wall_info &= ~0x88;					//a前壁は存在するはずがないので削除する
	if(!MF.FLAG.SCND || !MF2.FLAG.GOAL)write_map();							//a壁情報を地図に記入
*/

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();
	adv_pos();

	//====a歩数マップ・経路作成====
	if(!MF.FLAG.SCND)write_map();											//a壁情報を地図に記入
	r_cnt = 0;																//a経路カウンタの初期化
	make_smap();															//a歩数マップ作成
	make_route();															//a最短経路探索（route配列に動作が格納される）

	//====aマウス位置保存用変数を宣言
	uint8_t x, y;															//X，Y座標

	//====a探索走行====
	do{
		//----a進行----
		switch(route[r_cnt++]){												//route配列によって進行を決定。経路カウンタを進める
			//----a前進----
			case 0x88:
				if(route[r_cnt] == 0x88 && MF.FLAG.ACCL2){
					x = mouse.x;
					y = mouse.y;
					adv_pos();
					if((map[mouse.y][mouse.x] & 0x0f) == (map[mouse.y][mouse.x]>>4)){
						if(!MF2.FLAG.HACCEL){
							one_sectionA();
							MF2.FLAG.HACCEL = 1;
						}else{
							one_sectionU();
						}
					}else if(MF2.FLAG.HACCEL){
						one_sectionD();
						MF2.FLAG.HACCEL = 0;
					}else{
						one_sectionU();
					}
					mouse.x = x;
					mouse.y = y;
				}else if(MF2.FLAG.HACCEL){
					one_sectionD();
					MF2.FLAG.HACCEL = 0;
				}else{
					one_sectionU();
				}
				break;
			//----a右折スラローム----
			case 0x44:
				slalom_R90();
				break;
			//----180回転----
			case 0x22:
				half_sectionD();
				rotate_180();
				if(wall_info & 0x88){
					set_position();
				}else{
					half_sectionA();
				}
				break;
			//----a左折スラローム----
			case 0x11:
				slalom_L90();
				break;
		}
		adv_pos();
		if(!MF.FLAG.SCND)conf_route();

//	}while((mouse.x != goal_x) || (mouse.y != goal_y));
	}while(smap[mouse.y][mouse.x] != 0);

	for(int j=0; j<goal_mode-1; j++){
		one_sectionU();
		adv_pos();
	}
	half_sectionD();

	set_positionF();

	MF2.FLAG.GOAL = (MF2.FLAG.GOAL+1)%2;
	if(!MF2.FLAG.GOAL){
		HAL_Delay(500);
		rotate_180();										//180度回転
		driveC2(SETPOS_BACK);         						//a尻を当てる程度に後退。回転後に停止する
		degree_z = target_degree_z;
		start_mode = 0;
		goal_mode = 1;
	}

	if(!MF.FLAG.SCND){
		store_map_in_eeprom();
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//searchC2
//aスラローム走行でgoal座標に進む　重ね探索用
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchC2(){

	if(MF.FLAG.SRC2){
		load_map_from_eeprom();
	}

	//====aスタート位置壁情報取得====
	if(!MF.FLAG.SCND)get_wall_info();										//a壁情報の初期化, 後壁はなくなる
	if(!MF.FLAG.SCND)wall_info &= ~0x88;									//a前壁は存在するはずがないので削除する
	if(!MF.FLAG.SCND)write_map();											//a壁情報を地図に記入

	if(MF2.FLAG.GOAL){
		HAL_Delay(500);
		rotate_180();											//180度回転
//		driveC2(SETPOS_BACK);         							//a尻を当てる程度に後退。回転後に停止する
//		degree_z = target_degree_z;
//		start_mode = 0;
		start_mode = 1;
		goal_mode = 1;
	}

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();
	adv_pos();

	//====a歩数マップ・経路作成====
	if(!MF.FLAG.SCND)write_map();											//a壁情報を地図に記入
	r_cnt = 0;																//a経路カウンタの初期化
	make_smap();															//a歩数マップ作成
	make_route();															//a最短経路探索（route配列に動作が格納される）

	//====aマウス位置保存用変数を宣言
	uint8_t x, y;															//X，Y座標

	//====a探索走行====
	do{
		//----a進行----
		switch(route[r_cnt++]){												//route配列によって進行を決定。経路カウンタを進める
			//----a前進----
			case 0x88:
				if(route[r_cnt] == 0x88 && MF.FLAG.ACCL2){
					x = mouse.x;
					y = mouse.y;
					adv_pos();
					if((map[mouse.y][mouse.x] & 0x0f) == (map[mouse.y][mouse.x]>>4)){
						if(!MF2.FLAG.HACCEL){
							one_sectionA();
							MF2.FLAG.HACCEL = 1;
						}else{
							one_sectionU();
						}
					}else if(MF2.FLAG.HACCEL){
						one_sectionD();
						MF2.FLAG.HACCEL = 0;
					}else{
						one_sectionU();
					}
					mouse.x = x;
					mouse.y = y;
				}else if(MF2.FLAG.HACCEL){
					one_sectionD();
					MF2.FLAG.HACCEL = 0;
				}else{
					one_sectionU();
				}
				break;
			//----a右折スラローム----
			case 0x44:
				slalom_R90();
				break;
			//----180回転----
			case 0x22:
				half_sectionD();
				rotate_180();
				if(wall_info & 0x88){
					set_position();
				}else{
					half_sectionA();
				}
				break;
			//----a左折スラローム----
			case 0x11:
				slalom_L90();
				break;
		}
		adv_pos();
		if(!MF.FLAG.SCND)conf_route();

//	}while((mouse.x != goal_x) || (mouse.y != goal_y));
	}while(smap[mouse.y][mouse.x] != 0);

	for(int j=0; j<goal_mode-1; j++){
		one_sectionU();
		adv_pos();
	}
	half_sectionD();

	set_positionF();

	MF2.FLAG.GOAL = (MF2.FLAG.GOAL+1)%2;
	if(!MF2.FLAG.GOAL){
		HAL_Delay(500);
		rotate_180();										//180度回転
		driveC2(SETPOS_BACK);         						//a尻を当てる程度に後退。回転後に停止する
		degree_z = target_degree_z;
		start_mode = 0;
		goal_mode = 1;
	}

	if(!MF.FLAG.SCND){
		store_map_in_eeprom();
	}
}


/*-----------------------------------------------------------
		足立法探索走行D（スラローム+既知区間加速走行）
-----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//searchD
//aスラローム走行+既知区間加速でgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchD(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}

	//====aスタート位置壁情報取得====
//	if(!MF.FLAG.SCND)get_wall_info();										//a壁情報の初期化, 後壁はなくなる
//	if(!MF.FLAG.SCND)wall_info &= ~0x88;									//a前壁は存在するはずがないので削除する
//	if(!MF.FLAG.SCND)write_map();											//a壁情報を地図に記入

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();
	adv_pos();

	//====a歩数マップ・経路作成====
//	if(!MF.FLAG.SCND)write_map();											//a壁情報を地図に記入
	r_cnt = 0;												//a経路カウンタの初期化
	make_smap();											//a歩数マップ作成
	make_route();											//a最短経路探索（route配列に動作が格納される）

	MF2.FLAG.HACCEL = 0;

	//====a探索走行====
	do{
		//----a進行----
		switch(route[r_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//----a前進----
			case 0x88:
				if(MF.FLAG.SCND && MF.FLAG.ACCL2){
					if(((route[r_cnt-1] & route[r_cnt]) == 0x88) && (route[r_cnt] != 0xff) && (MF2.FLAG.HACCEL == 0)){
						one_sectionA();
						MF2.FLAG.HACCEL = 1;
					}
					else if((route[r_cnt] & 0x55) && (MF2.FLAG.HACCEL)){
						one_sectionD();
						MF2.FLAG.HACCEL = 0;
					}else{
						one_sectionU();
					}
				}else{
					one_sectionU();
				}
				break;
			//----a右折スラローム----
			case 0x44:
				slalom_R90();
				break;
			//----180回転----
			case 0x22:
				half_sectionD();
				rotate_180();
				if(wall_info & 0x88){
					set_position();
				}else{
					half_sectionA();
				}
				break;
			//----a左折スラローム----
			case 0x11:
				slalom_L90();
				break;
		}
		adv_pos();
//		if(!MF.FLAG.SCND)conf_route();

//	}while((mouse.x != goal_x) || (mouse.y != goal_y));
	}while(smap[mouse.y][mouse.x] != 0);

	for(int j=0; j<goal_mode-1; j++){
		one_sectionU();
		adv_pos();
	}
	half_sectionD();
	set_positionF();

	HAL_Delay(500);
	rotate_180();											//180度回転
//	driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
//	degree_z = target_degree_z;
//	start_mode = 0;
	start_mode = 1;
	goal_mode = 1;

	if(!MF.FLAG.SCND){
		store_map_in_eeprom();
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//searchE
//a未知壁を含むマスを仮goal座標とし、スラローム連続走行で全マスに進む。仮goalを連続的に変化させる。全面探索終了後は半区画減速
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchE(){

	int i = 0;
	int j = 0;
	do {
		if(i == 0){
			//====a前に壁が無い想定で問答無用で前進====
			half_sectionA();
			adv_pos();
			write_map();														//地図の初期化

			//====a歩数等初期化====
			m_step = r_cnt = 0;													//歩数と経路カウンタの初期化
			find_pregoal();														//仮goalまでの歩数マップの初期化
			make_smap2();
			make_route();														//最短経路探索(route配列に動作が格納される)
		}

		//====a探索走行====
		do {
			//----a進行----
			switch (route[r_cnt++]) {										//route配列によって進行を決定。経路カウンタを進める
				//----a前進----
			case 0x88:
				one_sectionU();
				break;
				//----a右折----
			case 0x44:
				slalom_R90();
				break;
				//----180回転----
			case 0x22:
				half_sectionD();
				rotate_180();
				if(wall_info & 0x88){
					set_position();
				}else{
					half_sectionA();
				}
				break;
				//----a左折----
			case 0x11:
				slalom_L90();
				break;
			}
			adv_pos();														//aマイクロマウス内部位置情報でも前進処理
			j++;
			if (j > 150) break;												//a移動マス数が250以上になった場合全面探索を中止

		} while ((mouse.x != pregoal_x) || (mouse.y != pregoal_y));			//a現在座標と仮goal座標が等しくなるまで実行

		get_wall_info();													//a壁情報の初期化, 後壁はなくなる
		write_map();														//a地図の初期化
		//printf("get pregoal, x = %d, y = %d\n", mouse.x, mouse.y);

		//====a歩数等初期化====
		m_step = r_cnt = 0;													//a歩数と経路カウンタの初期化

		find_pregoal();														//a仮goalまでの歩数マップの初期化
		if(MF2.FLAG.ALLMAP) {
			//printf("get MF2.FLAG.ALLMAP\n");
			half_sectionD();
			break;
		}
		make_smap2();
		make_route();														//a最短経路探索(route配列に動作が格納される)

		if (j > 150) {
			break;															//a移動マス数が250以上になった場合全面探索を中止
		}
		i++;

	} while (i < 150);														//a仮goalへの到着が130回以上になった場合全面探索を中止

	HAL_Delay(500);
	rotate_180();											//180度回転

	if(!MF.FLAG.SCND){
		store_map_in_eeprom();
	}
}


/*-----------------------------------------------------------
		足立法探索走行F（スラローム(+既知区間加速走行)+pass圧縮）
-----------------------------------------------------------*/
//+++++++++++++++++++++++++++++++++++++++++++++++
//searchF
//aスラローム走行(+既知区間加速)+pass圧縮でgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
/*void searchF(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}

	//====a1区画前進====
	adv_pos();

	//====a歩数マップ・経路作成====
	make_smap();											//a歩数マップ作成
	make_route();											//a最短経路探索（route配列に動作が格納される）

	//====pass圧縮====
	p_cnt = 0;												//a経路カウンタの初期化
	pass_route();

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();

	MF2.FLAG.HACCEL = 0;

	//====a探索走行====
	do{
		//----a進行----
		switch(pass[p_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//----a右スラローム----
			case -1:
				slalom_R90();
				break;

			//----a左スラローム----
			case -2:
				slalom_L90();
				break;

			//----a大回り右90----
			case -3:
				if(pass_mode == 1){
					half_sectionU();
					Lslalom_R90();
					half_sectionU();
				}else if(pass_mode == 2){
					if(pass[p_cnt] == -3){
						half_sectionU();
						Lslalom_R90();
						Lslalom_R90();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -4){
						half_sectionU();
						Lslalom_R90();
						Lslalom_L90();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -5){
						half_sectionU();
						Lslalom_R90();
						Lslalom_R180();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -6){
						half_sectionU();
						Lslalom_R90();
						Lslalom_L180();
						half_sectionU();
						p_cnt++;
					}else{
						half_sectionU();
						Lslalom_R90();
						half_sectionU();
					}
				}
				break;

			//----a大回り左90----
			case -4:
				if(pass_mode == 1){
					half_sectionU();
					Lslalom_L90();
					half_sectionU();
				}else if(pass_mode == 2){
					if(pass[p_cnt] == -3){
						half_sectionU();
						Lslalom_L90();
						Lslalom_R90();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -4){
						half_sectionU();
						Lslalom_L90();
						Lslalom_L90();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -5){
						half_sectionU();
						Lslalom_L90();
						Lslalom_R180();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -6){
						half_sectionU();
						Lslalom_L90();
						Lslalom_L180();
						half_sectionU();
						p_cnt++;
					}else{
						half_sectionU();
						Lslalom_L90();
						half_sectionU();
					}
				}
				break;

			//----a大回り右180----
			case -5:
				half_sectionU();
				Lslalom_R180();
				half_sectionU();
				break;

			//----a大回り左180----
			case -6:
				half_sectionU();
				Lslalom_L180();
				half_sectionU();
				break;

			//----pass配列最後(なお本来呼び出される前にゴールする)----
			case -114:
				rotate_180();
				while(1);
				break;

			//----a前進----
			default:
				if(pass[p_cnt-1] == 1){
					one_sectionU();
				}else{
					one_sectionA();
					MF2.FLAG.HACCEL = 1;
					int k;
					for(k = 0; k < pass[p_cnt-1]-2; k++){
						one_sectionU();
					}
					one_sectionD();
					MF2.FLAG.HACCEL = 0;
					full_led_write(BLUE);
				}
				break;
		}
		adv_pos2(pass[p_cnt-1]);

	}while((mouse.x != goal_x) || (mouse.y != goal_y));

	half_sectionD();

	HAL_Delay(500);
	rotate_180();											//180度回転

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//searchF2
//aスラローム(+既知区間加速探索走行)+pass圧縮+機体方向&位置未更新でgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchF2(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}

	//====a1区画前進====
	adv_pos();

	//====a歩数マップ・経路作成====
	make_smap();											//a歩数マップ作成
	make_route();											//a最短経路探索（route配列に動作が格納される）

	//====pass圧縮====
	p_cnt = 0;												//a経路カウンタの初期化
	pass_route();

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();

	MF2.FLAG.HACCEL = 0;

	//====a探索走行====
	do{
		//----a進行----
		switch(pass[p_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//----a右スラローム----
			case -1:
				slalom_R90();
				break;

			//----a左スラローム----
			case -2:
				slalom_L90();
				break;

			//----a大回り右90----
			case -3:
				if(pass_mode == 1){
					half_sectionU();
					Lslalom_R90();
					half_sectionU();
				}else if(pass_mode == 2){
					if(pass[p_cnt] == -3){
						half_sectionU();
						Lslalom_R90();
						Lslalom_R90();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -4){
						half_sectionU();
						Lslalom_R90();
						Lslalom_L90();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -5){
						half_sectionU();
						Lslalom_R90();
						Lslalom_R180();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -6){
						half_sectionU();
						Lslalom_R90();
						Lslalom_L180();
						half_sectionU();
						p_cnt++;
					}else{
						half_sectionU();
						Lslalom_R90();
						half_sectionU();
					}
				}
				break;

			//----a大回り左90----
			case -4:
				if(pass_mode == 1){
					half_sectionU();
					Lslalom_L90();
					half_sectionU();
				}else if(pass_mode == 2){
					if(pass[p_cnt] == -3){
						half_sectionU();
						Lslalom_L90();
						Lslalom_R90();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -4){
						half_sectionU();
						Lslalom_L90();
						Lslalom_L90();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -5){
						half_sectionU();
						Lslalom_L90();
						Lslalom_R180();
						half_sectionU();
						p_cnt++;
					}else if(pass[p_cnt] == -6){
						half_sectionU();
						Lslalom_L90();
						Lslalom_L180();
						half_sectionU();
						p_cnt++;
					}else{
						half_sectionU();
						Lslalom_L90();
						half_sectionU();
					}
				}
				break;

			//----a大回り右180----
			case -5:
				half_sectionU();
				Lslalom_R180();
				half_sectionU();
				break;

			//----a大回り左180----
			case -6:
				half_sectionU();
				Lslalom_L180();
				half_sectionU();
				break;

			//----pass配列最後(なお本来呼び出される前にゴールする)----
			case -114:
				rotate_180();
				rotate_180();
				while(1);
				break;

			//----a前進----
			default:
				if(pass[p_cnt-1] == 1){
					one_sectionU();
				}else{
					one_sectionA();
					MF2.FLAG.HACCEL = 1;
					int k;
					for(k = 0; k < pass[p_cnt-1]-2; k++){
						one_sectionU();
					}
					one_sectionD();
					MF2.FLAG.HACCEL = 0;
					full_led_write(BLUE);
				}
				break;
		}
//		adv_pos2(pass[p_cnt-1]);

	}while(pass[p_cnt] != -114);

	mouse.x = goal_x;
	mouse.y = goal_y;

	half_sectionD();

	HAL_Delay(500);
	rotate_180();											//180度回転

}
*/

//+++++++++++++++++++++++++++++++++++++++++++++++
//searchF3
//aスラローム(+既知区間加速探索走行)+pass圧縮+機体方向&位置未更新+半区画ベースでgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchF3(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}
	pass_mode = 3;

	//====a1区画前進====
	adv_pos();

	//====a歩数マップ・経路作成====
	make_smap();											//a歩数マップ作成
	make_route();											//a最短経路探索（route配列に動作が格納される）

	//====pass圧縮====
	p_cnt = 0;												//a経路カウンタの初期化
	pass_route2();

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();

	MF2.FLAG.HACCEL = 0;

	//====a探索走行====
	do{
		//----a進行----
		switch(pass[p_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//----a右スラローム----
			case -1:
				slalom_R90();
				break;

			//----a左スラローム----
			case -2:
				slalom_L90();
				break;

			//----a大回り右90----
			case -3:
				Lslalom_R90();
				break;

			//----a大回り左90----
			case -4:
				Lslalom_L90();
				break;

			//----a大回り右180----
			case -5:
				Lslalom_R180();
				break;

			//----a大回り左180----
			case -6:
				Lslalom_L180();
				break;

			//----pass配列最後(なお本来呼び出される前にゴールする)----
			case -114:
				rotate_180();
				rotate_180();
				while(1);
				break;

			//----a前進----
			default:
				if(pass[p_cnt-1] < 4){
					for(int k = 0; k < pass[p_cnt-1]; k++){
						half_sectionU();
					}
				}else{
					one_sectionA();
					MF2.FLAG.HACCEL = 1;
					int k;
					for(k = 0; k < pass[p_cnt-1]-4; k++){
						half_sectionU();
					}
					one_sectionD();
					MF2.FLAG.HACCEL = 0;
				}
				break;
		}
	}while(pass[p_cnt] != -114);

	mouse.x = goal_x;
	mouse.y = goal_y;

	for(int j=0; j<goal_mode-1; j++){
		one_sectionU();
		adv_pos();
	}
	half_sectionD();
	set_positionF();

	HAL_Delay(500);
	rotate_180();											//180度回転
//	driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
//	degree_z = target_degree_z;
//	start_mode = 0;
	start_mode = 1;
	goal_mode = 1;

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//searchF4
//aスラローム(+既知区間加速探索走行)+pass圧縮+機体方向&位置未更新+半区画ベース+斜め走行でgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchF4(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}
	pass_mode = 4;

	//====a1区画前進====
	adv_pos();

	//====a歩数マップ・経路作成====
	full_led_write1(RED);
	make_smap();											//a歩数マップ作成
	make_route();											//a最短経路探索（route配列に動作が格納される）

	//====pass圧縮====
	p_cnt = 0;												//a経路カウンタの初期化
	pass_route3();

	if(start_mode == 0 || start_mode == 1){					//a大回りではない場合、先頭の半区画直進をスキップ
		pass[p_cnt]--;
	}
	mouse.dir = mouse.dir * 2;

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();

	MF2.FLAG.HACCEL = 0;
	MF.FLAG.XDIR = 1;
	//====a探索走行====
	do{
		//----a進行----
		switch(pass[p_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//----a右スラローム----
			case -1:
				slalom_R90();
				break;

			//----a左スラローム----
			case -2:
				slalom_L90();
				break;

			//----a大回り右90----
			case -3:
				Lslalom_R90();
				break;

			//----a大回り左90----
			case -4:
				Lslalom_L90();
				break;

			//----a大回り右180----
			case -5:
				Lslalom_R180();
				break;

			//----a大回り左180----
			case -6:
				Lslalom_L180();
				break;

			//----a斜め右V45----
			case -7:
				v_R45();
				break;

			//----a斜め左V45----
			case -8:
				v_L45();
				break;

			//----a斜め右V90----
			case -9:
				v_R90();
				break;

			//----a斜め左V90----
			case -10:
				v_L90();
				break;

			//----a斜め右V135----
			case -11:
				v_R135();
				break;

			//----a斜め左V135----
			case -12:
				v_L135();
				break;

			//----a斜め右V45Goal----
			case -13:
				v_R45D();
				break;

			//----a斜め左V45Goal----
			case -14:
				v_L45D();
				break;

			//----a斜め右V135Goal----
			case -15:
				v_R135D();
				break;

			//----a斜め左V135Goal----
			case -16:
				v_L135D();
				break;

			//----pass配列最後(なお本来呼び出される前にゴールする)----
			case -114:
				rotate_180();
				rotate_180();
				while(1);
				break;

			//----a前進----
			default:
				if(pass[p_cnt-1] < 4){
					for(int k = 0; k < pass[p_cnt-1]; k++){
						half_sectionU();
					}
				}else if(pass[p_cnt-1] < 64){
					one_sectionA();
					MF2.FLAG.HACCEL = 1;
					int k;
					for(k = 0; k < pass[p_cnt-1]-4; k++){
						half_sectionU();
					}
					one_sectionD();
					MF2.FLAG.HACCEL = 0;
				}else{
					if((pass[p_cnt] == 64) && (pass[p_cnt+1] == 64) && (pass[p_cnt+2] == 64) && (MF2.FLAG.HACCEL == 0)){
						one_sectionVA();
						MF2.FLAG.HACCEL = 1;
						p_cnt++;
					}
					else if((pass[p_cnt] == 64) && (pass[p_cnt+1] != 64) && (MF2.FLAG.HACCEL == 1)){
						one_sectionVD();
						MF2.FLAG.HACCEL = 0;
						p_cnt++;
					}else{
						half_sectionV();
					}
				}
				break;
		}

	}while(pass[p_cnt] != -114);

	mouse.x = goal_x;
	mouse.y = goal_y;
	mouse.dir = mouse.dir / 2;
	MF.FLAG.XDIR = 0;

	if(pass[p_cnt-1] != -13 && pass[p_cnt-1] != -14 && pass[p_cnt-1] != -15 && pass[p_cnt-1] != -16){
		for(int j=0; j<goal_mode-1; j++){
			one_sectionU();
			adv_pos();
		}
		half_sectionD();
		set_positionF();
	}else{														//a減速斜めでgoalする場合
		for(int j=0; j<goal_mode-1; j++){
			one_section();
			adv_pos();
		}
	}
	HAL_Delay(500);
	rotate_180();											//180度回転
//	driveC2(SETPOS_BACK); 							        //a尻を当てる程度に後退。回転後に停止する
//	degree_z = target_degree_z;
//	start_mode = 0;
	start_mode = 1;
	goal_mode = 1;
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//searchF5
//aスラローム(+既知区間加速探索走行)+pass圧縮+機体方向&位置未更新+半区画ベース+大回り減速でgoal座標に進む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void searchF5(){

	if(MF.FLAG.SCND){
		load_map_from_eeprom();
	}
	pass_mode = 3;

	//====a1区画前進====
	adv_pos();

	//====a歩数マップ・経路作成====
	make_smap();											//a歩数マップ作成
	make_route();											//a最短経路探索（route配列に動作が格納される）

	//====pass圧縮====
	p_cnt = 0;												//a経路カウンタの初期化
	pass_route2();

	//====a前に壁が無い想定で問答無用で前進====
	start_sectionA();

	MF2.FLAG.HACCEL = 0;

	//====a探索走行====
	do{
		//----a進行----
		switch(pass[p_cnt++]){								//route配列によって進行を決定。経路カウンタを進める
			//----a右スラローム----
			case -1:
				slalom_R90();
				break;

			//----a左スラローム----
			case -2:
				slalom_L90();
				break;

			//----a大回り右90----
			case -3:
				Lslalom_R90();
				break;

			//----a大回り左90----
			case -4:
				Lslalom_L90();
				break;

			//----a大回り右180----
			case -5:
				Lslalom_R180();
				break;

			//----a大回り左180----
			case -6:
				Lslalom_L180();
				break;

			//----pass配列最後(なお本来呼び出される前にゴールする)----
			case -114:
				rotate_180();
				rotate_180();
				while(1);
				break;

			//----a前進----
			default:
				if(pass[p_cnt-1] < 4){
					for(int k = 0; k < pass[p_cnt-1]; k++){
						half_sectionU();
					}
				}else{
					one_sectionA();
					MF2.FLAG.HACCEL = 1;
					int k;
					for(k = 0; k < pass[p_cnt-1]-4; k++){
						half_sectionU();
					}
					one_sectionD();
					MF2.FLAG.HACCEL = 0;
				}
				break;
		}
	}while(pass[p_cnt] != -114);

	mouse.x = goal_x;
	mouse.y = goal_y;

	for(int j=0; j<goal_mode-1; j++){
		one_sectionU();
		adv_pos();
	}
	half_sectionD();
	set_positionF();

	HAL_Delay(500);
	rotate_180();											//180度回転
//	driveC2(SETPOS_BACK);         //a尻を当てる程度に後退。回転後に停止する
//	degree_z = target_degree_z;
//	start_mode = 0;
	start_mode = 1;
	goal_mode = 1;

}


//+++++++++++++++++++++++++++++++++++++++++++++++
//adv_pos
//aマイクロマウス内部位置情報で前進させる
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void adv_pos(){

	switch(mouse.dir){										//aマイクロマウスが現在向いている方向で判定
	case 0x00:												//a北方向に向いている場合
		mouse.y++;											//Y座標をインクリメント
		break;
	case 0x01:												//a東方向に向いている場合
		mouse.x++;											//X座標をインクリメント
		break;
	case 0x02:												//a南方向に向いている場合
		mouse.y--;											//Y座標をデクリメント
		break;
	case 0x03:												//a西方向に向いている場合
		mouse.x--;											//X座標をデクリメント
		break;
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//adv_pos2
//aマイクロマウス内部位置情報で前進させる(pass圧縮対応版)
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void adv_pos2(int8_t pass_pat){
	int k;

	switch(mouse.dir){										//aマイクロマウスが現在向いている方向で判定
	case 0x00:												//a北方向に向いている場合
//		switch(pass_pat){										//aマイクロマウスが現在向いている方向で判定
		switch(pass[p_cnt-1]){										//aマイクロマウスが現在向いている方向で判定
		//----a左右スラローム----
		case -1:
		case -2:
			mouse.y++;											//Y座標をインクリメント
			break;

		//----a大回り右90----
		case -3:
			mouse.x--;											//X座標をデクリメント
			mouse.y++;											//Y座標をインクリメント
			mouse.y++;											//Y座標をインクリメント
			break;

		//----a大回り左90----
		case -4:
			mouse.x++;											//X座標をインクリメント
			mouse.y++;											//Y座標をインクリメント
			mouse.y++;											//Y座標をインクリメント
			break;

		//----a大回り右180----
		case -5:
			mouse.y--;											//Y座標をデクリメント
			mouse.x--;											//X座標をデクリメント
			mouse.y++;											//Y座標をインクリメント
			mouse.y++;											//Y座標をインクリメント
			break;

		//----a大回り左180----
		case -6:
			mouse.y--;											//Y座標をデクリメント
			mouse.x++;											//X座標をインクリメント
			mouse.y++;											//Y座標をインクリメント
			mouse.y++;											//Y座標をインクリメント
			break;

		//----a前進----
		default:
			for(k = 0; k < pass_pat; k++){
				mouse.y++;											//Y座標をインクリメント
			}
			break;
		}
		break;

	case 0x01:												//a東方向に向いている場合
		switch(pass_pat){										//aマイクロマウスが現在向いている方向で判定
		//----a右スラローム----
		case -1:
		case -2:
			mouse.x++;											//X座標をインクリメント
			break;

		//----a大回り右90----
		case -3:
			mouse.y++;											//Y座標をインクリメント
			mouse.x++;											//X座標をインクリメント
			mouse.x++;											//X座標をインクリメント
			break;

		//----a大回り左90----
		case -4:
			mouse.y--;											//Y座標をデクリメント
			mouse.x++;											//X座標をインクリメント
			mouse.x++;											//X座標をインクリメント
			break;

		//----a大回り右180----
		case -5:
			mouse.x--;											//X座標をデクリメント
			mouse.y++;											//Y座標をインクリメント
			mouse.x++;											//X座標をインクリメント
			mouse.x++;											//X座標をインクリメント
			break;

		//----a大回り左180----
		case -6:
			mouse.x--;											//X座標をデクリメント
			mouse.y--;											//Y座標をデクリメント
			mouse.x++;											//X座標をインクリメント
			mouse.x++;											//X座標をインクリメント
			break;

		//----a前進----
		default:
			for(k = 0; k < pass[p_cnt-1]; k++){
				mouse.x++;											//X座標をインクリメント
			}
			break;
		}
		break;

	case 0x02:												//a南方向に向いている場合
		switch(pass_pat){										//aマイクロマウスが現在向いている方向で判定
		//----a右スラローム----
		case -1:
		case -2:
			mouse.y--;											//Y座標をデクリメント
			break;

		//----a大回り右90----
		case -3:
			mouse.x++;											//X座標をインクリメント
			mouse.y--;											//Y座標をデクリメント
			mouse.y--;											//Y座標をデクリメント
			break;

		//----a大回り左90----
		case -4:
			mouse.x--;											//Y座標をデクリメント
			mouse.y--;											//Y座標をデクリメント
			mouse.y--;											//Y座標をデクリメント
			break;

		//----a大回り右180----
		case -5:
			mouse.y++;											//Y座標をインクリメント
			mouse.x++;											//X座標をインクリメント
			mouse.y--;											//Y座標をデクリメント
			mouse.y--;											//Y座標をデクリメント
			break;

		//----a大回り左180----
		case -6:
			mouse.y++;											//Y座標をインクリメント
			mouse.x--;											//X座標をデクリメント
			mouse.y--;											//Y座標をデクリメント
			mouse.y--;											//Y座標をデクリメント
			break;

		//----a前進----
		default:
			for(k = 0; k < pass[p_cnt-1]; k++){
				mouse.y--;											//Y座標をデクリメント
			}
			break;
		}
		break;

	case 0x03:												//a西方向に向いている場合
		switch(pass_pat){										//aマイクロマウスが現在向いている方向で判定
		//----a右スラローム----
		case -1:
		case -2:
			mouse.x--;											//X座標をデクリメント
			break;

		//----a大回り右90----
		case -3:
			mouse.y--;											//Y座標をデクリメント
			mouse.x--;											//X座標をデクリメント
			mouse.x--;											//X座標をデクリメント
			break;

		//----a大回り左90----
		case -4:
			mouse.y++;											//Y座標をインクリメント
			mouse.x--;											//X座標をデクリメント
			mouse.x--;											//X座標をデクリメント
			break;

		//----a大回り右180----
		case -5:
			mouse.x++;											//X座標をインクリメント
			mouse.y--;											//Y座標をデクリメント
			mouse.x--;											//X座標をデクリメント
			mouse.x--;											//X座標をデクリメント
			break;

		//----a大回り左180----
		case -6:
			mouse.x++;											//X座標をインクリメント
			mouse.y++;											//Y座標をインクリメント
			mouse.x--;											//X座標をデクリメント
			mouse.x--;											//X座標をデクリメント
			break;

		//----a前進----
		default:
			for(k = 0; k < pass[p_cnt-1]; k++){
				mouse.x--;											//X座標をデクリメント
			}
			break;
		}
		break;
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//conf_route
//a進路を判定する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void conf_route(){

	//----a壁情報書き込み----
	write_map();

	//----a最短経路上に壁があれば進路変更----
	if(wall_info & route[r_cnt]){
		make_smap();										//a歩数マップを更新
		make_route();										//a最短経路を更新
		r_cnt = 0;											//a経路カウンタを0に
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//map_Init
//aマップ格納配列map[][]の初期化をする
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void map_Init(){

	//====a変数宣言====
	uint8_t x, y;											//for文用変数

	//====a初期化開始====
	//aマップのクリア
	for(y = 0; y <= 15; y++){								//a各Y座標で実行
		for(x = 0; x <= 15; x++){							//a各X座標で実行
			map[y][x] = 0xf0;								//a上位4ビット（2次走行時）を壁あり，下位4ビット（1次走行時）を壁なしとする。
		}
	}

	//a確定壁の配置
	for(y = 0; y <= 15; y++){								//a各Y座標で実行
		map[y][0] |= 0xf1;									//a最西に壁を配置
		map[y][15] |= 0xf4;									//a最東に壁を配置
	}
	for(x = 0; x <= 15; x++){								//a各X座標で実行
		map[0][x] |= 0xf2;									//a最南に壁を配置
		map[15][x] |= 0xf8;									//a最北に壁を配置
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//write_map
//aマップデータを書き込む
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void write_map(){

	//====a変数宣言====
	uint8_t m_temp;											//a向きを補正した壁情報

	//====a壁情報の補正格納====
	m_temp = (wall_info >> mouse.dir) & 0x0f;				//aセンサ壁情報をmouse.dirで向きを補正させて下位4bit分を残す
	m_temp |= (m_temp << 4);								//a上位4bitに下位4bitをコピー。この作業でm_tempにNESW順で壁が格納

	//====aデータの書き込み====
	map[mouse.y][mouse.x] = m_temp; 						//a現在地に壁情報書き込み
	//----a周辺に書き込む----
	//a北側について
	if(mouse.y != 15){										//a現在最北端でないとき
		if(m_temp & 0x88){									//a北壁がある場合
			map[mouse.y + 1][mouse.x] |= 0x22;				//a北側の区画から見て南壁ありを書き込む
		}else{												//a北壁がない場合
			map[mouse.y + 1][mouse.x] &= 0xDD;				//a北側の区画から見て南壁なしを書き込む
		}
	}
	//a東側について
	if(mouse.x != 15){										//a現在最東端でないとき
		if(m_temp & 0x44){									//a東壁がある場合
			map[mouse.y][mouse.x + 1] |= 0x11;				//a東側の区画から見て西壁ありを書き込む
		}else{												//a北壁がない場合
			map[mouse.y][mouse.x + 1] &= 0xEE;				//a東側の区画から見て西壁なしを書き込む
		}
	}
	//a南壁について
	if(mouse.y != 0){										//a現在最南端でないとき
		if(m_temp & 0x22){									//a南壁がある場合
			map[mouse.y - 1][mouse.x] |= 0x88;				//a南側の区画から見て北壁ありを書き込む
		}else{												//a南壁がない場合
			map[mouse.y - 1][mouse.x] &= 0x77;				//a南側の区画から見て北壁なしを書き込む
		}
	}
	//a西側について
	if(mouse.x != 0){										//a現在最西端でないとき
		if(m_temp & 0x11){									//a西壁がある場合
			map[mouse.y][mouse.x - 1] |= 0x44;				//a西側の区画から見て東壁ありを書き込む
		}else{												//a西壁がない場合
			map[mouse.y][mouse.x - 1] &= 0xBB;				//a西側の区画から見て東側なしを書き込む
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//turn_dir
//aマウスの方向を変更する
//a引数1：t_pat …… 回転方向(search.hでマクロ定義)
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_dir(uint8_t t_pat, uint8_t t_mode){

	//====a方向を変更====
	if(t_mode < 2){												//4方位モード
		mouse.dir = (mouse.dir + t_pat) & 0x03;					//a指定された分mouse.dirを回転させる
		if(t_mode == 1){
			if(t_pat == 0x01) target_degree_z -= 90;			//a目標角度+右90度
			if(t_pat == 0xff) target_degree_z += 90;			//a目標角度+左90度
			if(t_pat == 0x02) target_degree_z -= 180;			//a目標角度+右180度
		}
	}else{														//8方位モード
		mouse.dir = (mouse.dir + t_pat) & 0x07;					//a指定された分mouse.dirを回転させる
		if(t_mode == 3){
			if(t_pat == 0x01) target_degree_z -= 45;			//a目標角度+右90度
			if(t_pat == 0x02) target_degree_z -= 90;			//a目標角度+左90度
			if(t_pat == 0x03) target_degree_z -= 135;			//a目標角度+右180度
			if(t_pat == 0x04) target_degree_z -= 180;			//a目標角度+右180度
			if(t_pat == 0xff) target_degree_z += 45;			//a目標角度+右90度
			if(t_pat == 0xfe) target_degree_z += 90;			//a目標角度+左90度
			if(t_pat == 0xfd) target_degree_z += 135;			//a目標角度+右180度
			if(t_pat == 0xfc) target_degree_z += 180;			//a目標角度+右180度
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//make_smap
//a歩数マップを作成する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void make_smap(void){

	//====a変数宣言====
	uint8_t x, y;											//for文用変数
	uint8_t m_temp_sample[16];

	//====a歩数マップのクリア====
	for(y = 0; y <= 15; y++){								//a各Y座標で実行
		for(x = 0; x <= 15; x++){							//a各X座標で実行
			smap[y][x] = 0x03e7;							//a未記入部分は歩数最大とする
		}
	}

	//====aゴール座標を0にする====
	m_step = 0;												//a歩数カウンタを0にする

	for(int j=0; j<goal_mode; j++){
		for(int k=0; k<goal_mode; k++){
		smap[goal_y+j][goal_x+k] = 0;
		}
	}

//	smap[goal_y][goal_x] = 0;

	//====a歩数カウンタの重みづけ====
	int straight = 2;
	int turn = 5;
	full_led_write1(GREEN);
	//====a自分の座標にたどり着くまでループ====
	do{
		//----aマップ全域を捜索----
		for(y = 0; y <= 15; y++){							//a各Y座標で実行
			for(x = 0; x <= 15; x++){						//a各X座標で実行
				//----a現在最大の歩数を発見したとき----
				if(smap[y][x] == m_step){					//a歩数カウンタm_stepの値が現在最大の歩数
					uint8_t m_temp = map[y][x];				//map配列からマップデータを取り出す
					if(MF.FLAG.SCND){						//a二次走行用のマップを作成する場合（二次走行時はMF.FLAG.SCNDが立っている）
						m_temp >>= 4;						//a上位4bitを使うので4bit分右にシフトさせる
					}
					//----a北壁についての処理----
					if(!(m_temp & 0x08) && y != 15){		//a北壁がなく現在最北端でないとき
						if(smap[y+1][x] >= (smap[y][x]+turn)){		//a北側が記入後より大きいなら
							smap[y+1][x] = smap[y][x] + turn;		//a次の歩数を書き込む
							if(MF.FLAG.STRAIGHT){
								//----a直線優先処理----
								for(int k=1; k<16-y; k++) {					//a現在座標から見て北のマスすべてにおいて
									m_temp_sample[k] = map[y+k][x];				//map配列からマップデータを取り出す
									if(MF.FLAG.SCND) m_temp_sample[k] >>= 4;		//a二次走行用のマップを作成する場合上位4bitを使うので4bit分右にシフトさせる
									if(!(m_temp_sample[k] & 0x08) && (y+k) != 15) {		//a北壁がなく現在最北端でないとき
										if(smap[y+k+1][x] >= (smap[y+k][x]+straight)){		//a北側が記入後より大きいなら
											smap[y+k+1][x] = smap[y+k][x] + straight;		//a直線分インクリメントした値を次のマスの歩数マップに書き込む
										}
									}
									else break;
								}
							}
						}
					}
					//----a東壁についての処理----
					if(!(m_temp & 0x04) && x != 15){		//a東壁がなく現在最東端でないとき
						if(smap[y][x+1] >= (smap[y][x]+turn)){	//a東側が記入後より大きいなら
							smap[y][x+1] = smap[y][x] + turn;	//a次の歩数を書き込む
							if(MF.FLAG.STRAIGHT){
								//----a直線優先処理----
								for(int k=1; k<16-x; k++) {					//a現在座標から見て東のマスすべてにおいて
									m_temp_sample[k] = map[y][x+k];				//map配列からマップデータを取り出す
									if(MF.FLAG.SCND) m_temp_sample[k] >>= 4;		//a二次走行用のマップを作成する場合上位4bitを使うので4bit分右にシフトさせる
									if(!(m_temp_sample[k] & 0x04) && (x+k) != 15) {		//a東壁がなく現在最東端でないとき
										if(smap[y][x+k+1] >= (smap[y][x+k]+straight)){		//a東側が記入後より大きいなら
											smap[y][x+k+1] = smap[y][x+k] + straight;		//a直線分インクリメントした値を次のマスの歩数マップに書き込む
										}
									}
									else break;
								}
							}
						}
					}
					//----a南壁についての処理----
					if(!(m_temp & 0x02) && y != 0){			//a南壁がなく現在最南端でないとき
						if(smap[y-1][x] >= (smap[y][x]+turn)){	//a南側が記入後より大きいなら
							smap[y-1][x] = smap[y][x] + turn;	//a次の歩数を書き込む
							if(MF.FLAG.STRAIGHT){
								//----a直線優先処理----
								for(int k=1; k<y; k++) {						//a現在座標から見て南のマスすべてにおいて
									m_temp_sample[k] = map[y-k][x];				//map配列からマップデータを取り出す
									if(MF.FLAG.SCND) m_temp_sample[k] >>= 4;		//a二次走行用のマップを作成する場合上位4bitを使うので4bit分右にシフトさせる
									if(!(m_temp_sample[k] & 0x02) && (y-k) != 0) {		//a南壁がなく現在最南端でないとき
										if(smap[y-k-1][x] >= (smap[y-k][x]+straight)){		//a南側が記入後より大きいなら
											smap[y-k-1][x] = smap[y-k][x] + straight;		//a直線分インクリメントした値を次のマスの歩数マップに書き込む
										}
									}
									else break;
								}
							}
						}
					}
					//----a西壁についての処理----
					if(!(m_temp & 0x01) && x != 0){			//a西壁がなく現在最西端でないとき
						if(smap[y][x-1] >= (smap[y][x]+turn)){	//a西側が記入後より大きいなら
							smap[y][x-1] = smap[y][x] + turn;	//a次の歩数を書き込む
							if(MF.FLAG.STRAIGHT){
								//----a直線優先処理----
								for(int k=1; k<x; k++) {						//a現在座標から見て西のマスすべてにおいて
									m_temp_sample[k] = map[y][x-k];				//map配列からマップデータを取り出す
									if(MF.FLAG.SCND) m_temp_sample[k] >>= 4;		//a二次走行用のマップを作成する場合上位4bitを使うので4bit分右にシフトさせる
									if(!(m_temp_sample[k] & 0x01) && (x-k) != 0) {		//a西壁がなく現在最西端でないとき
										if(smap[y][x-k-1] >= (smap[y][x-k]+straight)){		//a西側が記入後より大きいなら
											smap[y][x-k-1] = smap[y][x-k] + straight;		//a直線分インクリメントした値を次のマスの歩数マップに書き込む
										}
									}
									else break;
								}
							}
						}
					}
				}
			}
		}
		//====a歩数カウンタのインクリメント====
		m_step++;
	}while(smap[mouse.y][mouse.x] == 0x03e7);					//a現在座標が未記入ではなくなるまで実行
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//make_route
//a最短経路を導出する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void make_route(){

	//====a変数宣言====
	uint8_t x, y;												//X，Y座標
	uint8_t dir_temp =  mouse.dir;								//aマウスの方角を表すmouse.dirの値をdir_temp変数に退避させる

	//====a最短経路を初期化====
	uint16_t i;
	for(i = 0; i < 256; i++){
		route[i] = 0xff;										//routeを0xffで初期化
	}

	//====a歩数カウンタをセット====
	uint16_t m_step = smap[mouse.y][mouse.x];					//a現在座標の歩数マップ値を取得

	//====x, yに現在座標を書き込み====
	x = mouse.x;
	y = mouse.y;

	//====a最短経路を導出====
	if(pass_mode == 4){
		route[0] = 0x77;
		i = 1;
	}else{
		i = 0;
	}

	do{
		uint8_t m_temp = map[y][x];								//a比較用マップ情報の格納
		if(MF.FLAG.SCND){										//a二次走行用のマップを作成する場合（二次走行時はMF.FLAG.SCNDが立っている）
			m_temp >>= 4;										//a上位4bitを使うので4bit分右にシフトさせる
		}
		//----a北を見る----
		if(!(m_temp & 0x08) && (smap[y+1][x] < m_step)){		//a北側に壁が無く、現在地より小さい歩数マップ値であれば
			route[i] = (0x00 - mouse.dir) & 0x03;				//route配列に進行方向を記録
			m_step = smap[y+1][x];								//a最大歩数マップ値を更新
			y++;												//a北に進んだのでY座標をインクリメント
		}
		//----a東を見る----
		else if(!(m_temp & 0x04) && (smap[y][x+1] < m_step)){	//a東側に壁が無く、現在地より小さい歩数マップ値であれば
			route[i] = (0x01 - mouse.dir) & 0x03;				//route配列に進行方向を記録
			m_step = smap[y][x+1];								//a最大歩数マップ値を更新
			x++;												//a東に進んだのでX座標をインクリメント
		}
		//----a南を見る----
		else if(!(m_temp & 0x02) && (smap[y-1][x] < m_step)){	//a南側に壁が無く、現在地より小さい歩数マップ値であれば
			route[i] = (0x02 - mouse.dir) & 0x03;				//route配列に進行方向を記録
			m_step = smap[y-1][x];								//a最大歩数マップ値を更新
			y--;												//a南に進んだのでY座標をデクリメント
		}
		//----a西を見る----
		else if(!(m_temp & 0x01) && (smap[y][x-1] < m_step)){	//a西側に壁が無く、現在地より小さい歩数マップ値であれば
			route[i] = (0x03 - mouse.dir) & 0x03;				//route配列に進行方向を記録
			m_step = smap[y][x-1];								//a最大歩数マップ値を更新
			x--;												//a西に進んだのでX座標をデクリメント
		}

		if(run_dir == 1){
			//----a東を見る----
	/*		if(!(m_temp & 0x04) && (smap[y][x+1] < m_step)){		//a東側に壁が無く、現在地より小さい歩数マップ値であれば
				route[i] = (0x01 - mouse.dir) & 0x03;				//route配列に進行方向を記録
				m_step = smap[y][x+1];								//a最大歩数マップ値を更新
				x++;												//a東に進んだのでX座標をインクリメント
			}
			//----a北を見る----
			else if(!(m_temp & 0x08) && (smap[y+1][x] < m_step)){	//a北側に壁が無く、現在地より小さい歩数マップ値であれば
				route[i] = (0x00 - mouse.dir) & 0x03;				//route配列に進行方向を記録
				m_step = smap[y+1][x];								//a最大歩数マップ値を更新
				y++;												//a北に進んだのでY座標をインクリメント
			}
			//----a西を見る----
			else if(!(m_temp & 0x01) && (smap[y][x-1] < m_step)){	//a西側に壁が無く、現在地より小さい歩数マップ値であれば
				route[i] = (0x03 - mouse.dir) & 0x03;				//route配列に進行方向を記録
				m_step = smap[y][x-1];								//a最大歩数マップ値を更新
				x--;												//a西に進んだのでX座標をデクリメント
			}
			//----a南を見る----
			else if(!(m_temp & 0x02) && (smap[y-1][x] < m_step)){	//a南側に壁が無く、現在地より小さい歩数マップ値であれば
				route[i] = (0x02 - mouse.dir) & 0x03;				//route配列に進行方向を記録
				m_step = smap[y-1][x];								//a最大歩数マップ値を更新
				y--;												//a南に進んだのでY座標をデクリメント
			}*/

		}
		//----a格納データ形式変更----
		switch(route[i]){										//route配列に格納した要素値で分岐
		case 0x00:												//a前進する場合
			if(pass_mode < 3){
				route[i] = 0x88;									//a格納データ形式を変更
			}else{
				route[i] = 0x77;
				route[i+1] = 0x77;
				i++;
			}
			break;
		case 0x01:												//a右折する場合
			turn_dir(DIR_TURN_R90, 0);							//a内部情報の方向を90度右回転
			route[i] = 0x44;									//a格納データ形式を変更
			break;
		case 0x02:												//Uターンする場合
			turn_dir(DIR_TURN_180, 0);							//a内部情報の方向を180度回転
			route[i] = 0x22;									//a格納データ形式を変更
			break;
		case 0x03:												//a左折する場合
			turn_dir(DIR_TURN_L90, 0);							//a内部情報の方向を90度左回転
			route[i] = 0x11;									//a格納データ形式を変更
			break;
		default:												//aそれ以外の場合
			route[i] = 0x00;									//a格納データ形式を変更
			break;
		}
		i++;													//aカウンタをインクリメント
	}while(smap[y][x] != 0);									//a進んだ先の歩数マップ値が0（=ゴール）になるまで実行

	if(MF.FLAG.SCND){
		goal_x = x;
		goal_y = y;												//a二次走行でgoal後自己座標をgoal座標にするのでその時用
	}
	mouse.dir = dir_temp;										//dir_tempに退避させた値をmouse.dirにリストア
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//find_pregoal
//	未探索壁を含む＆現在座標から最も近いマス(=仮goal)を探す
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void find_pregoal()
{
	//====変数宣言====
	uint8_t x, y;															//for文用変数
	uint8_t m_temp;															//マップデータ一時保持
	//uint8_t m_temp_sample[16];
	uint8_t break_flag = 0;													//未知壁マスを見つけた時のループ脱出フラグ

	//====歩数マップのクリア====
	for (y = 0; y <= 0x0f; y++) {											//各Y座標で実行
		for (x = 0; x <= 0x0f; x++) {										//各X座標で実行
			smap[y][x] = 0x03e7;											//未記入部分は歩数最大とする
		}
	}

	//====探索完了フラグのクリア====
	MF2.FLAG.ALLMAP = 0;

	//====現在座標を0にする====
	smap[mouse.y][mouse.x] = 0;

	//====歩数カウンタを0にする====
	m_step = 0;																//現在記入した最大の歩数となる

	//====歩数カウンタの重みづけ====
	int straight = 3;
	int turn = 1;

	//====自分の座標にたどり着くまでループ====
	do {
		//----マップ全域を捜索----
		for (y = 0; y <= 0x0f; y++) {										//各Y座標で実行
			for (x = 0; x <= 0x0f; x++) {									//各X座標で実行
				//----現在最大の歩数を発見したとき----
				if (smap[y][x] == m_step) {									//歩数格納変数m_stepの値が現在最大の歩数のとき
					m_temp = map[y][x];										//map配列からマップデータを取り出す
					//----北壁についての処理----
					if (!(m_temp & 0x08) && y != 0x0f) {					//北壁がなく現在最北端でないとき
						if (smap[y + 1][x] == 0x03e7) {						//北側が未記入なら
							smap[y + 1][x] = smap[y][x] + turn;				//曲線分インクリメントした値を次のマスの歩数マップに書き込む
							if (((map[y + 1][x] & 0x0f ) << 4) != (map[y + 1][x] & 0xf0)) {		//map内の上位4bitと下位4bitが異なる場合
								break_flag = 1;								//for文を抜けるフラグを立てて
								m_step2 = smap[y + 1][x];					//仮goalの歩数を記録
								pregoal_x = x;
								pregoal_y = y + 1;							//仮goalの座標を記録
								break;
							}
						}
					}
					//----東壁についての処理----
					if (!(m_temp & 0x04) && x != 0x0f) {					//東壁がなく現在最東端でないとき
						if (smap[y][x + 1] == 0x03e7) {						//東側が未記入なら
							smap[y][x + 1] = smap[y][x] + turn;				//曲線分インクリメントした値を次のマスの歩数マップに書き込む
							if (((map[y][x + 1] & 0x0f) << 4) != (map[y][x + 1] & 0xf0)) {		//map内の上位4bitと下位4bitが異なる場合
								break_flag = 1;								//for文を抜けるフラグを立てて
								m_step2 = smap[y][x + 1];					//仮ゴールの歩数を記録
								pregoal_x = x + 1;
								pregoal_y = y;								//仮goalの座標を記録
								break;
							}
						}
					}
					//----南壁についての処理----
					if (!(m_temp & 0x02) && y != 0) {						//南壁がなく現在最南端でないとき
						if (smap[y - 1][x] == 0x03e7) {						//南側が未記入なら
							smap[y - 1][x] = smap[y][x] + turn;				//曲線分インクリメントした値を次のマスの歩数マップに書き込む
							if (((map[y - 1][x] & 0x0f) << 4) != (map[y - 1][x] & 0xf0)) {		//map内の上位4bitと下位4bitが異なる場合
								break_flag = 1;								//for文を抜けるフラグを立てて
								m_step2 = smap[y - 1][x];					//仮ゴールの歩数を記録
								pregoal_x = x;
								pregoal_y = y - 1;							//仮goalの座標を記録
								break;
							}
						}
					}
					//----西壁についての処理----
					if (!(m_temp & 0x01) && x != 0) {						//西壁がなく現在最西端でないとき
						if (smap[y][x - 1] == 0x03e7) {						//西側が未記入なら
							smap[y][x - 1] = smap[y][x] + turn;				//次の歩数を書き込む
							if (((map[y][x - 1] & 0x0f) << 4) != (map[y][x - 1] & 0xf0)) {		//map内の上位4bitと下位4bitが異なる場合
								break_flag = 1;								//for文を抜けるフラグを立てて
								m_step2 = smap[y][x - 1];					//仮ゴールの歩数を記録
								pregoal_x = x - 1;
								pregoal_y = y;								//仮goalの座標を記録
								break;							}
						}
					}
				}
			}
			if (break_flag) break;		//map内の上位4bitと下位4bitが異なる場合、for文を抜ける
		}
		//====歩数カウンタのインクリメント====
		m_step++;
		if(m_step > 500) MF2.FLAG.ALLMAP = 1;
	} while(break_flag == 0 && MF2.FLAG.ALLMAP != 1);		//未探索壁ありマスを見つけるまで実行
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//make_smap2
//	歩数マップを作成する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void make_smap2()
{
	//====変数宣言====
	uint8_t x, y;															//for文用変数
	uint8_t m_temp;															//マップデータ一時保持

	//====歩数マップのクリア====
	for (y = 0; y <= 0x0f; y++) {											//各Y座標で実行
		for (x = 0; x <= 0x0f; x++) {										//各X座標で実行
			smap[y][x] = 0x03e7;											//未記入部分は歩数最大とする
		}
	}

	//====仮ゴール座標を0にする====
	smap[pregoal_y][pregoal_x] = 0;

	//====歩数カウンタを0にする====
	m_step = 0;																//現在記入した最大の歩数となる

	//====歩数カウンタの重みづけ====
	int straight = 3;
	int turn = 5;

	//====自分の座標にたどり着くまでループ====
	do {
		//----マップ全域を捜索----
		for (y = 0; y <= 0x0f; y++) {										//各Y座標で実行
			for (x = 0; x <= 0x0f; x++) {									//各X座標で実行
				//----現在最大の歩数を発見したとき----
				if (smap[y][x] == m_step) {									//歩数格納変数m_stepの値が現在最大の歩数のとき
					m_temp = map[y][x];										//map配列からマップデータを取り出す
					//----北壁についての処理----
					if (!(m_temp & 0x08) && y != 0x0f) {					//北壁がなく現在最北端でないとき
						if (smap[y + 1][x] == 0x03e7) {						//北側が未記入なら
							smap[y + 1][x] = smap[y][x] + turn;				//曲線分インクリメントした値を次のマスの歩数マップに書き込む
						}
					}
					//----東壁についての処理----
					if (!(m_temp & 0x04) && x != 0x0f) {					//東壁がなく現在最東端でないとき
						if (smap[y][x + 1] == 0x03e7) {						//東側が未記入なら
							smap[y][x + 1] = smap[y][x] + turn;				//曲線分インクリメントした値を次のマスの歩数マップに書き込む
						}
					}
					//----南壁についての処理----
					if (!(m_temp & 0x02) && y != 0) {						//南壁がなく現在最南端でないとき
						if (smap[y - 1][x] == 0x03e7) {						//南側が未記入なら
							smap[y - 1][x] = smap[y][x] + turn;				//曲線分インクリメントした値を次のマスの歩数マップに書き込む
						}
					}
					//----西壁についての処理----
					if (!(m_temp & 0x01) && x != 0) {						//西壁がなく現在最西端でないとき
						if (smap[y][x - 1] == 0x03e7) {						//西側が未記入なら
							smap[y][x - 1] = smap[y][x] + turn;				//次の歩数を書き込む
						}
					}
				}
			}
		}
		//====歩数カウンタのインクリメント====
		m_step++;
	} while (smap[mouse.y][mouse.x] == 0x03e7);		//現在座標が未記入ではなくなるまで実行
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//pass_route
// route配列をpass圧縮する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
/*void pass_route(void){
	int i;
	for(i = 0; i < 256; i++){
		pass[i] = 0;								//pass配列の初期化
	}
	uint8_t p = 0;									//pass配列の配列番号用変数
	i = 0;
	uint8_t s = 0;									//a直線数カウント用変数
	while(route[i-1] != 0xff){
		s = 0;
		switch(route[i]){
		case 0x88:										//aまず直進
			s++;
			switch(route[i+1]){
			case 0x88:									//a直進→直進
				while(route[i+2] == 0x88){				//aさらに直進であるならば
					s++;								//a直進カウンタ+1
					i++;
				}
				pass[p] = s;							//pass配列に直進数を代入
				i++;
				break;

			case 0x44:									//a直進→右
				switch(route[i+2]){
				case 0x88:								//a直進→右→直進　=　大回り右90度

					if(pass_mode == 1){
						pass[p] = -3;
						i = i + 3;
					}else if(pass_mode == 2){
						if(route[i+3] == 0x44 && route[i+4] == 0x88){
							pass[p] = -3;
							pass[p+1] = -3;
							i = i + 5;						//大回り右90→大回り右90
							p++;
						}else if(route[i+3] == 0x11 && route[i+4] == 0x88){
							pass[p] = -3;
							pass[p+1] = -4;
							i = i + 5;						//大回り右90→大回り左90
							p++;
						}else if(route[i+3] == 0x44 && route[i+4] == 0x44 && route[i+5] == 0x88){
							pass[p] = -3;
							pass[p+1] = -5;
							i = i + 6;						//大回り右90→大回り右180
							p++;
						}else if(route[i+3] == 0x11 && route[i+4] == 0x11 && route[i+5] == 0x88){
							pass[p] = -3;
							pass[p+1] = -6;
							i = i + 6;						//大回り右90→大回り左180
							p++;
						}else{
							pass[p] = -3;
							i = i + 3;
						}
					}
					break;

				case 0x44:								//a直進→右→右
					if(route[i+3] == 0x88){				//a直進→右→右→直進　=　大回り右180度
						pass[p] = -5;
						i = i + 4;
					}else{								//a直進→右→右→左(左以外は存在しない)
						if(p > 0 && pass[p-1] > 0){
							pass[p-1]++;
							pass[p] = -1;
							pass[p+1] = -1;
							pass[p+2] = -2;				//a直進→右→右→左(左以外は存在しない)　そのまま保存　直前が直進の場合
							p = p + 2;
						}else{
							pass[p] = 1;
							pass[p+1] = -1;
							pass[p+2] = -1;
							pass[p+3] = -2;				//a直進→右→右→左(左以外は存在しない)　そのまま保存
							p = p + 3;
						}
						i = i + 4;
					}
					break;

				case 0x11:								//a直進→右→左
					if(p > 0 && pass[p-1] > 0){
						pass[p-1]++;
						pass[p] = -1;
						pass[p+1] = -2;					//a直進→右→左　そのまま保存　　直前が直進の場合
						p++;
					}else{
						pass[p] = s;
						pass[p+1] = -1;
						pass[p+2] = -2;					//a直進→右→左　そのまま保存
						p = p + 2;
					}
					i = i + 3;
					break;

				default:								//a直進→右→終了
					if(p > 0 && pass[p-1] > 0){
						pass[p-1]++;
						pass[p] = -1;					//a直進→右→終了　　直前が直進の場合
					}else{
						pass[p] = s;
						pass[p+1] = -1;					//a直進→右→終了
						p++;
					}
					i = i + 2;
					break;
				}
				break;

			case 0x11:									//a直進→左
				switch(route[i+2]){
				case 0x88:								//a直進→左→直進　=　大回り左90度

					if(pass_mode == 1){
						pass[p] = -4;
						i = i + 3;
					}else if(pass_mode == 2){
						if(route[i+3] == 0x44 && route[i+4] == 0x88){
							pass[p] = -4;
							pass[p+1] = -3;
							i = i + 5;						//大回り右90→大回り右90
							p++;
						}else if(route[i+3] == 0x11 && route[i+4] == 0x88){
							pass[p] = -4;
							pass[p+1] = -4;
							i = i + 5;						//大回り右90→大回り左90
							p++;
						}else if(route[i+3] == 0x44 && route[i+4] == 0x44 && route[i+5] == 0x88){
							pass[p] = -4;
							pass[p+1] = -5;
							i = i + 6;						//大回り右90→大回り右180
							p++;
						}else if(route[i+3] == 0x11 && route[i+4] == 0x11 && route[i+5] == 0x88){
							pass[p] = -4;
							pass[p+1] = -6;
							i = i + 6;						//大回り右90→大回り左180
							p++;
						}else{
							pass[p] = -4;
							i = i + 3;
						}
					}
					break;

				case 0x44:								//a直進→左→右
					if(p > 0 && pass[p-1] > 0){
						pass[p-1]++;
						pass[p] = -2;
						pass[p+1] = -1;					//a直進→左→右　そのまま保存　　直前が直進の場合
						p++;
					}else{
						pass[p] = s;
						pass[p+1] = -2;
						pass[p+2] = -1;					//a直進→左→右　そのまま保存
						p = p + 2;
					}
					i = i + 3;
					break;

				case 0x11:								//a直進→左→左
					if(route[i+3] == 0x88){				//a直進→左→左→直進　=　大回り左180度
						pass[p] = -6;
						i = i + 4;
					}else{								//a直進→左→左→右(右以外は存在しない)
						if(p > 0 && pass[p-1] > 0){
							pass[p-1]++;
							pass[p] = -2;
							pass[p+1] = -2;
							pass[p+2] = -1;				//a直進→左→左→右(右以外は存在しない)　そのまま保存　　直前が直進の場合
							p = p + 2;
						}else{
							pass[p] = 1;
							pass[p+1] = -2;
							pass[p+2] = -2;
							pass[p+3] = -1;				//a直進→左→左→右(右以外は存在しない)　そのまま保存
							p = p + 3;
						}
						i = i + 4;
					}
					break;

				default:								//a直進→左→終了
					if(p > 0 && pass[p-1] > 0){
						pass[p-1]++;
						pass[p] = -2;					//a直進→左→終了　そのまま保存　　直前が直進の場合
					}else{
						pass[p] = s;
						pass[p+1] = -2;					//a直進→左→終了　そのまま保存
						p++;
					}
					i = i + 2;
					break;
				}
				break;

			default:									//a直進→終了
				if(p > 0 && pass[p-1] > 0){
					pass[p-1]++;						//a直進→終了　そのまま保存　　直前が直進の場合
				}else{
					pass[p] = s;						//a直進→終了　そのまま保存
				}
				i++;
			}
			break;

		case 0x44:										//a右　=　右スラローム
			pass[p] = -1;
			i++;
			break;

		case 0x11:										//a左　=　左スラローム
			pass[p] = -2;
			i++;
			break;

		case 0xff:										//a終了条件
			pass[p] = -114;
			i++;
			break;
		}
		p++;											//pass配列数カウンタ+1
	}
}
*/

//+++++++++++++++++++++++++++++++++++++++++++++++
//pass_route2
// route配列をpass圧縮する(半区画ベース)
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void pass_route2(void){
	int i;
	uint8_t s_flag = 0;
	for(i = 0; i < 256; i++){
		pass[i] = 0;								//pass配列の初期化
	}
	uint8_t p = 0;									//pass配列の配列番号用変数
	i = 0;
	uint8_t s = 0;									//a直線数カウント用変数
	while(route[i-1] != 0xff){
		s = 0;
		if(route[i] == 0x44){
			pass[p] = -1;							//a右スラローム
			i++;
		}else if(route[i] == 0x11){
			pass[p] = -2;							//a左スラローム
			i++;
		}else if(route[i] == 0x77 && route[i+1] == 0x44 && route[i+2] == 0x77){
			s_flag = 0;
			pass[p] = -3;							//a大回り右スラローム
			i = i + 3;
		}else if(route[i] == 0x77 && route[i+1] == 0x11 && route[i+2] == 0x77){
			s_flag = 0;
			pass[p] = -4;							//a大回り左スラローム
			i = i + 3;
		}else if(route[i] == 0x77 && route[i+1] == 0x44 && route[i+2] == 0x44 && route[i+3] == 0x77){
			s_flag = 0;
			pass[p] = -5;							//a大回り右180スラローム
			i = i + 4;
		}else if(route[i] == 0x77 && route[i+1] == 0x11 && route[i+2] == 0x11 && route[i+3] == 0x77){
			s_flag = 0;
			pass[p] = -6;							//a大回り左180スラローム
			i = i + 4;
		}else if(route[i] == 0xff){
			pass[p] = -114;							//a終了条件用
			i++;
		}else if(route[i] == 0x77){
			if(s_flag){
				pass[p-1]++;
				p--;
				s_flag = 0;
				i++;
			}else{
				s++;
				pass[p] = s;
				while(route[i+1] == 0x77){
					s_flag = 1;
					pass[p] = s;
					i++;
					s++;
				}
				if(!s_flag){
					i++;
				}
			}
		}
		p++;
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//pass_route3
// route配列をpass圧縮する(半区画ベース+斜めあり)
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void pass_route3(void){
	int i;
	uint8_t s_flag = 0;
	MF2.FLAG.V = 0;										//a斜めフラグの初期化
	for(i = 0; i < 256; i++){
		pass[i] = 0;								//pass配列の初期化
	}
	uint8_t p = 0;									//pass配列の配列番号用変数
	i = 0;
	uint8_t s = 0;									//a直線数カウント用変数
	while(route[i-1] != 0xff){
		s = 0;
		if(route[i] == 0x77 && route[i+1] == 0x44 && route[i+2] == 0x77){
			s_flag = 0;
			pass[p] = -3;							//a大回り右90
			if(i == 0){								//aスタート時の走行モード切り替えフラグ
				start_mode = 2;
			}
			i = i + 3;
		}else if(route[i] == 0x77 && route[i+1] == 0x11 && route[i+2] == 0x77){
			s_flag = 0;
			pass[p] = -4;							//a大回り左90
			if(i == 0){								//aスタート時の走行モード切り替えフラグ
				start_mode = 2;
			}
			i = i + 3;
		}else if(route[i] == 0x77 && route[i+1] == 0x44 && route[i+2] == 0x44 && route[i+3] == 0x77){
			s_flag = 0;
			pass[p] = -5;							//a大回り右180
			i = i + 4;
		}else if(route[i] == 0x77 && route[i+1] == 0x11 && route[i+2] == 0x11 && route[i+3] == 0x77){
			s_flag = 0;
			pass[p] = -6;							//a大回り左180
			i = i + 4;
		}else if(route[i] == 0x77 && route[i+1] == 0x44 && route[i+2] == 0x11){
			s_flag = 0;
			pass[p] = -7;							//a斜め右V45in
			MF2.FLAG.V = 1;
			if(i == 0){								//aスタート時の走行モード切り替えフラグ
				start_mode = 2;
			}
			i = i + 2;
		}else if(MF2.FLAG.V == 1 && route[i] == 0x44 && route[i+1] == 0x77){
			s_flag = 0;
			pass[p] = -7;							//a斜め右V45out
			MF2.FLAG.V = 0;
			i = i + 2;
		}else if(route[i] == 0x77 && route[i+1] == 0x11 && route[i+2] == 0x44){
			s_flag = 0;
			pass[p] = -8;							//a斜め左V45in
			MF2.FLAG.V = 1;
			i = i + 2;
		}else if(MF2.FLAG.V == 1 && route[i] == 0x11 && route[i+1] == 0x77){
			s_flag = 0;
			pass[p] = -8;							//a斜め左V45out
			MF2.FLAG.V = 0;
			i = i + 2;
		}else if(MF2.FLAG.V == 1 && route[i] == 0x44 && route[i+1] == 0x44 && route[i+2] == 0x11){
			s_flag = 0;
			pass[p] = -9;							//a斜め右V90
			MF2.FLAG.V = 1;
			i = i + 2;
		}else if(MF2.FLAG.V == 1 && route[i] == 0x11 && route[i+1] == 0x11 && route[i+2] == 0x44){
			s_flag = 0;
			pass[p] = -10;							//a斜め左V90
			MF2.FLAG.V = 1;
			i = i + 2;
		}else if(route[i] == 0x77 && route[i+1] == 0x44 && route[i+2] == 0x44 && route[i+3] == 0x11){
			s_flag = 0;
			pass[p] = -11;							//a斜め右V135in
			MF2.FLAG.V = 1;
			if(i == 0){								//aスタート時の走行モード切り替えフラグ
				start_mode = 2;
			}
			i = i + 3;
		}else if(MF2.FLAG.V == 1 && route[i] == 0x44 && route[i+1] == 0x44 && route[i+2] == 0x77){
			s_flag = 0;
			pass[p] = -11;							//a斜め右V135out
			MF2.FLAG.V = 0;
			i = i + 3;
		}else if(route[i] == 0x77 && route[i+1] == 0x11 && route[i+2] == 0x11 && route[i+3] == 0x44){
			s_flag = 0;
			pass[p] = -12;							//a斜め左V135in
			MF2.FLAG.V = 1;
			i = i + 3;
		}else if(MF2.FLAG.V == 1 && route[i] == 0x11 && route[i+1] == 0x11 && route[i+2] == 0x77){
			s_flag = 0;
			pass[p] = -12;							//a斜め左V135out
			MF2.FLAG.V = 0;
			i = i + 3;
		}else if(route[i] == 0xff){
			if(MF2.FLAG.V == 1){
				if(pass[p-1] == -1 && pass[p-2] == -1){
					pass[p-2] = -15;
					pass[p-1] = -114;
				}else if(pass[p-1] == -2 && pass[p-2] == -2){
					pass[p-2] = -16;
					pass[p-1] = -114;
				}else if(pass[p-1] == -1){
					pass[p-1] = -13;
					pass[p] = -114;
				}else if(pass[p-1] == -2){
					pass[p-1] = -14;
					pass[p] = -114;
				}
			}
			pass[p] = -114;							//a終了用配列
			i++;
		}else if(MF2.FLAG.V == 1 && route[i] == 0x44 && route[i+1] == 0x11){
			s_flag = 0;
			pass[p] = 64;							//a斜め半直線
			MF2.FLAG.V = 1;
			i++;
		}else if(MF2.FLAG.V == 1 && route[i] == 0x11 && route[i+1] == 0x44){
			s_flag = 0;
			pass[p] = 64;							//a斜め半直線
			MF2.FLAG.V = 1;
			i++;
		}else if(route[i] == 0x44){
			pass[p] = -1;							//a右スラローム
			i++;
		}else if(route[i] == 0x11){
			pass[p] = -2;							//a左スラローム
			i++;
		}else if(route[i] == 0x77){
			if(s_flag){
				pass[p-1]++;						//aスラローム前半直線追加
				p--;
				s_flag = 0;
				i++;
			}else{
				s++;
				pass[p] = s;						//aもし半直線1つだけの場合whileには入らない
				while(route[i+1] == 0x77){
					s_flag = 1;
					pass[p] = s;					//a半直線2つ以上続く場合、最後半直線は大回り用に残す
					i++;
					s++;
				}
				if(!s_flag){
					i++;
				}
			}
		}
		p++;
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//store_map_in_eeprom
// mapデータをeepromに格納する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void store_map_in_eeprom(void){
	printf("eprom func start \n");
	eeprom_enable_write();
	printf("eprom enable_write fin \n");
	int i;
	for(i = 0; i < 16; i++){
		int j;
		for(j = 0; j < 16; j++){
			eeprom_write_halfword(i*16 + j, (uint16_t) map[i][j]);
		}
	}
	eeprom_disable_write();
}


//+++++++++++++++++++++++++++++++++++++++++++++++
//load_map_in_eeprom
// mapデータをeepromから取得する
//a引数：なし
//a戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void load_map_from_eeprom(void){
	int i;
	for(i = 0; i < 16; i++){
		int j;
		for(j = 0; j < 16; j++){
			map[i][j] = (uint8_t) eeprom_read_halfword(i*16 + j);
		}
	}
}


