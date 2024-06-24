#include "TIMxPWM.h"


extern TIM_HandleTypeDef PWM_TIMx;
/*************************************************************
** Function name:       TIM1_Init
** Descriptions:        定时器1初始化
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
inline void TIMx_PWM_Init(void)
{
    SetTIM1Channel1HighLeaveTime_us(0);
    SetTIM1Channel2HighLeaveTime_us(0);
    SetTIM1Channel3HighLeaveTime_us(0);
		    
		TimerxChannel4ITEnable(0);
		CalculateAdcOffset();
}
/*************************************************************
** Function name:       Timer1ITEnable
** Descriptions:        定时一通道4比较中断使能
** Input parameters:    isEnable：1：使能 0：失能
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void TimerxChannel4ITEnable(uint8_t isEnable)
{
    if(isEnable == 1) {
			HAL_TIM_PWM_Start(&PWM_TIMx,PWM_CHANNEL1);
			HAL_TIM_PWM_Start(&PWM_TIMx,PWM_CHANNEL2);
			HAL_TIM_PWM_Start(&PWM_TIMx,PWM_CHANNEL3);

			HAL_TIMEx_PWMN_Start(&PWM_TIMx,PWM_CHANNEL1);
			HAL_TIMEx_PWMN_Start(&PWM_TIMx,PWM_CHANNEL2);
			HAL_TIMEx_PWMN_Start(&PWM_TIMx,PWM_CHANNEL3);
			
			HAL_TIM_PWM_Start_IT(&PWM_TIMx,PWM_CHANNEL4);
			HAL_TIM_Base_Start_IT(&PWM_TIMx);

    } else {
			HAL_TIM_PWM_Stop(&PWM_TIMx,PWM_CHANNEL1);
			HAL_TIM_PWM_Stop(&PWM_TIMx,PWM_CHANNEL2);
			HAL_TIM_PWM_Stop(&PWM_TIMx,PWM_CHANNEL3);

			HAL_TIMEx_PWMN_Stop(&PWM_TIMx,PWM_CHANNEL1);
			HAL_TIMEx_PWMN_Stop(&PWM_TIMx,PWM_CHANNEL2);
			HAL_TIMEx_PWMN_Stop(&PWM_TIMx,PWM_CHANNEL3);
			
			HAL_TIM_PWM_Stop_IT(&PWM_TIMx,PWM_CHANNEL4);
			HAL_TIM_Base_Stop_IT(&PWM_TIMx);
    }
}

/*************************************************************
** Function name:       SetTIM1ChannelxHighLeaveTime_us
** Descriptions:        设置定时器1通道1高电平时间（中央对齐模式二）
** Input parameters:    time：高电平时间 单位us 范围 0-PWM_LIMIT
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
inline void SetTIM1Channel1HighLeaveTime_us(int16_t time){

    __HAL_TIM_SET_COMPARE(&PWM_TIMx,PWM_CHANNEL1, PWM_LIMIT(time));
}

inline void SetTIM1Channel2HighLeaveTime_us(int16_t time){

    __HAL_TIM_SET_COMPARE(&PWM_TIMx,PWM_CHANNEL2, PWM_LIMIT(time));
}

inline void SetTIM1Channel3HighLeaveTime_us(int16_t time){

    __HAL_TIM_SET_COMPARE(&PWM_TIMx,PWM_CHANNEL3, PWM_LIMIT(time));
}

inline void SetTIM1Channel4HighLeaveTime_us(int16_t time){

    __HAL_TIM_SET_COMPARE(&PWM_TIMx,PWM_CHANNEL4, PWM_LIMIT(time));
}

