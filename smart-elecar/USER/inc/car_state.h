#ifndef _CAR_STATE_H_
#define _CAR_STATE_H_



//extern car jiangxing_No_1;//车体发车状态

typedef struct // 定义pid常量参数及输出变量
{

	int start_flag;//发车标志位
	int stop_flag;//停车标志位
	int start_mode;//停车标志位
} car_states;

extern car_states jiangxing_No_1;//车体发车状态

#endif