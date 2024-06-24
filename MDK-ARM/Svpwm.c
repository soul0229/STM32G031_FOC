#include "Svpwm.h"
#include <string.h>
#include "tim.h"
#include "usart.h"


extern char uart_tx[23];

SVPWM_EXPORT(gMotor,LIM_PWM_OUT,MAX_PWM_OUT,ADC_DELAY_TIM,9.0f,
                SetTIM1Channel1HighLeaveTime_us,
                SetTIM1Channel2HighLeaveTime_us,
                SetTIM1Channel3HighLeaveTime_us,
								SetTIM1Channel4HighLeaveTime_us)

/*************************************************************
** Function name:       SectorJudgment
** Descriptions:        扇区判断,利用uα和uβ来判断扇区
** Input parameters:    pSvpwm：结构体名字
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void SvpwmSectorJudgment(PSvpwm_Struct pSvpwm)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;
    uint8_t sector;
		float u_alpha_sqrt3_2, u_beta, u_beta_2;
		u_alpha_sqrt3_2 = pSvpwm->u_alpha * FLOAT_SQRT3_2;
		u_beta = pSvpwm->u_beta;
		u_beta_2 = u_beta/2;

    pSvpwm->u1 = u_beta;
		pSvpwm->u2 = u_alpha_sqrt3_2 - u_beta_2;
		pSvpwm->u3 = -u_alpha_sqrt3_2 - u_beta_2;


    if (pSvpwm->u1 > 0) {
        a = 1;
    } else {
        a = 0;
    }
    if (pSvpwm->u2 > 0) {
        b = 1;
    } else {
        b = 0;
    }
    if (pSvpwm->u3 > 0) {
        c = 1;
    } else {
        c = 0;
    }

   sector = 4*c + 2*b + a;
    switch (sector) {
        case 3:
            pSvpwm->sector = 1;
            break;
        case 1:
            pSvpwm->sector = 2;
            break;
        case 5:
            pSvpwm->sector = 3;
            break;
        case 4:
            pSvpwm->sector = 4;
            break;
        case 6:
            pSvpwm->sector = 5;
            break;
        case 2:
            pSvpwm->sector = 6;
            break;
    }
}
/*************************************************************
** Function name:       GetVectorDuration
** Descriptions:        获取矢量作用时长
** Input parameters:    pSvpwm:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void GetVectorDuration(PSvpwm_Struct pSvpwm)
{
	int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;
	
    switch (pSvpwm->sector) {
        case 1:
            pSvpwm->t4 = FLOAT_SQRT3 * ts / udc * pSvpwm->u2;
            pSvpwm->t6 = FLOAT_SQRT3 * ts / udc * pSvpwm->u1;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t4 - pSvpwm->t6) / 2;
            break;
        case 2:
            pSvpwm->t2 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u2;
            pSvpwm->t6 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u3;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t2 - pSvpwm->t6) / 2;
            break;
        case 3:
            pSvpwm->t2 = FLOAT_SQRT3 * ts / udc * pSvpwm->u1;
            pSvpwm->t3 = FLOAT_SQRT3 * ts / udc * pSvpwm->u3;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t2 - pSvpwm->t3) / 2;
            break;
        case 4:
            pSvpwm->t1 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u1;
            pSvpwm->t3 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u2;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t1 - pSvpwm->t3) / 2;
            break;
        case 5:
            pSvpwm->t1 = FLOAT_SQRT3 * ts / udc * pSvpwm->u3;
            pSvpwm->t5 = FLOAT_SQRT3 * ts / udc * pSvpwm->u2;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t1 - pSvpwm->t5) / 2;
            break;
        case 6:
            pSvpwm->t4 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u3;
            pSvpwm->t5 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u1;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t4 - pSvpwm->t5) / 2;
            break;
        default:
            break;
    }
}
/*************************************************************
** Function name:       SvpwmGenerate
** Descriptions:        SVPWM生成周期函数
** Input parameters:    pSvpwm:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/

void SvpwmGenerate(PSvpwm_Struct pSvpwm)
{
	int16_t *t[3] = {
		&FOCdata.svpwm[0], 
		&FOCdata.svpwm[1], 
		&FOCdata.svpwm[2],
	};
	
    switch (pSvpwm->sector) {
        case 1:
            *t[0] = pSvpwm->t4 + pSvpwm->t6 + pSvpwm->t7;
            *t[1] = pSvpwm->t6 + pSvpwm->t7;
            *t[2] = pSvpwm->t7;
            break;
        case 2:
            *t[0] = pSvpwm->t6 + pSvpwm->t7;
            *t[1] = pSvpwm->t2 + pSvpwm->t6 + pSvpwm->t7;
            *t[2] = pSvpwm->t7;
            break;
        case 3:
            *t[0] = pSvpwm->t7;
            *t[1] = pSvpwm->t2 + pSvpwm->t3 + pSvpwm->t7;
            *t[2] = pSvpwm->t3 + pSvpwm->t7;
            break;
        case 4:
            *t[0] = pSvpwm->t7;
            *t[1] = pSvpwm->t3 + pSvpwm->t7;
            *t[2] = pSvpwm->t1 + pSvpwm->t3 + pSvpwm->t7;
            break;
        case 5:
            *t[0] = pSvpwm->t5 + pSvpwm->t7;
            *t[1] = pSvpwm->t7;
            *t[2] = pSvpwm->t1 + pSvpwm->t5 + pSvpwm->t7;
            break;
        case 6:
            *t[0] = pSvpwm->t4 + pSvpwm->t5 + pSvpwm->t7;
            *t[1] = pSvpwm->t7;
            *t[2] = pSvpwm->t5 + pSvpwm->t7;
            break;
    }
    pSvpwm->SetChannelAHighLeaveTime_us(*t[0]);
    pSvpwm->SetChannelBHighLeaveTime_us(*t[1]);
    pSvpwm->SetChannelCHighLeaveTime_us(*t[2]);
}
/*************************************************************
** Function name:       SvpwmContorol
** Descriptions:        Svpwm控制
** Input parameters:    pSvpwm:结构体指针
**                      uα：阿尔法轴目标电压
**                      uβ：贝塔轴目标电压
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void SvpwmContorol(float u_alpha,float u_beta)
{
    gMotor.u_alpha = u_alpha;
    gMotor.u_beta = u_beta;
    //1.扇区判断
    SvpwmSectorJudgment(&gMotor);
    //1.计算矢量作用时长
    GetVectorDuration(&gMotor);
    //1.SVPWM生成
    SvpwmGenerate(&gMotor);
}
/*************************************************************
** Function name:       GetSVPWMSector
** Descriptions:        获取SVPWM扇区
** Input parameters:    None
** Output parameters:   None
** Returned value:      SVPWM扇区(1-6)
** Remarks:             None
*************************************************************/
uint8_t GetSVPWMSector(void)
{
    return gMotor.sector;
}


