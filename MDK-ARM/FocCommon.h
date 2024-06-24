#ifndef __FOC_COMMON_H
#define __FOC_COMMON_H
#include "main.h"
#include "stdint.h"

#define PWM_CHANNEL1 		TIM_CHANNEL_1
#define PWM_CHANNEL2 		TIM_CHANNEL_2
#define PWM_CHANNEL3 		TIM_CHANNEL_3
#define PWM_CHANNEL4 		TIM_CHANNEL_4

#define MAX_PWM_OUT			6400
#define LIM_PWM_OUT			4800
#define MIN_PWM_OUT			0
#define ADC_DELAY_TIM		50
#define PWM_TIMx				htim1

#define LIMIT(x,min,max)		((x>max)?max:((x<=min)?min:x))

#define PWM_LIMIT(x)		LIMIT(x,MIN_PWM_OUT,LIM_PWM_OUT)

#define FLOAT_SQRT3 		1.732050808f
#define FLOAT_SQRT3_2 	0.866025404f
#define FLOAT_SQRT3_3 	0.577350269f
#define IDIQ_MAX 				FLOAT_SQRT3_3


#define ADC_PORT				hadc1
#define ADC_FILTER_KP 	0.8f
#define RESIST_NUM 			3
#define POWER_ADC_NUM 	1
#define ADC_NUM 				(RESIST_NUM + POWER_ADC_NUM)

#define POPLE_PAIRS			4

#include "TIMxPWM.h"
#include "Svpwm.h"
#include "FOC.h"
#include "sincos.h"
//#include "arm_math.h"


typedef struct data_struct{
	uint16_t tx_TAIL:8;
	uint16_t tx_HEADER:8;
	uint16_t tx_ID:8;
	uint16_t hall1:8;
	uint16_t hall2:8;
	uint16_t hall3:8;
	int16_t svpwm[3];
	int16_t adc[3];
	int16_t speed[2];
	#define TX_SIZE 22
}FOC_data;

#define DATA_TX_INIT {.tx_TAIL = 0x02, .tx_HEADER 	= 0xaa, .tx_ID = 0x01, }

extern FOC_data FOCdata;

#endif