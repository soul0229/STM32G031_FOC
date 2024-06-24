#include "TIMxPWM.h"


extern TIM_HandleTypeDef PWM_TIMx;
/*************************************************************
** Function name:       TIM1_Init
** Descriptions:        ��ʱ��1��ʼ��
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
** Descriptions:        ��ʱһͨ��4�Ƚ��ж�ʹ��
** Input parameters:    isEnable��1��ʹ�� 0��ʧ��
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
** Descriptions:        ���ö�ʱ��1ͨ��1�ߵ�ƽʱ�䣨�������ģʽ����
** Input parameters:    time���ߵ�ƽʱ�� ��λus ��Χ 0-PWM_LIMIT
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

