#include "FOC.h"
#define FOC_ANGLE_TO_RADIN 0.01745f
#define M_OUTMAX  9.0f * 0.577f
#define M_KP  0.0008f
#define M_KI  0.0004f
#define M_KD  0.0f


FOC_data FOCdata = DATA_TX_INIT;
int16_t lastAdcData[RESIST_NUM] = {0, 0, 0};

static void GetMotorPreCurrent(float *ua,float *ub,float *uc);

//声明FOC对象
FOC_EXPORT(gMotorFOC,POPLE_PAIRS,RESIST_NUM,
			TimerxChannel4ITEnable ,
			NULL,
			GetSVPWMSector,
			GetMotorPreCurrent,
			SvpwmContorol)
/*************************************************************
** Function name:       CurrentReconstruction
** Descriptions:        电流重构
** Input parameters:    pFOC:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void CurrentReconstruction(PFOC_Struct pFOC)
{
	pFOC->GetPreCurrent(&pFOC->ia, &pFOC->ib, &pFOC->ic);
	if (pFOC->iNum < 3) {
        return;
    }
    switch (pFOC->GetSVPWMSector()) {
        case 1:
            pFOC->ia =0.0f - pFOC->ib - pFOC->ic;
            break;
        case 2:
            pFOC->ib =0.0f - pFOC->ia - pFOC->ic;
            break;
        case 3:
            pFOC->ib =0.0f - pFOC->ia - pFOC->ic;
            break;
        case 4:
            pFOC->ic =0.0f - pFOC->ia - pFOC->ib;
            break;
        case 5:
            pFOC->ic =0.0f - pFOC->ia - pFOC->ib;
            break;
        case 6:
            pFOC->ia =0.0f - pFOC->ib - pFOC->ic;
            break;
        default:
            break;
    }
}
/*************************************************************
** Function name:       ClarkeTransform
** Descriptions:        Clarke正变换
** Input parameters:    pFOC:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void ClarkeTransform(PFOC_Struct pFOC)
{
    pFOC->ialpha = pFOC->ia;
    pFOC->ibeta = (pFOC->ia + 2.0f * pFOC->ib) / FLOAT_SQRT3;
}
/*************************************************************
** Function name:       ParkTransform
** Descriptions:        Park正变换
** Input parameters:    pFOC:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
static void ParkTransform(PFOC_Struct pFOC)
{
    pFOC->id = pFOC->ialpha * cos_table(pFOC->radian) + pFOC->ibeta * sin_table(pFOC->radian);
    pFOC->iq = -pFOC->ialpha * sin_table(pFOC->radian) + pFOC->ibeta * cos_table(pFOC->radian);
}

/*************************************************************
** Function name:       ParkAntiTransform
** Descriptions:        Park反变换
** Input parameters:    pFOC:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
static void ParkAntiTransform(PFOC_Struct pFOC)
{
    pFOC->ialphaSVPWM = pFOC->idPID.out * cos_table(pFOC->radian) - pFOC->iqPID.out * sin_table(pFOC->radian);
    pFOC->ibetaSVPWM = pFOC->idPID.out * sin_table(pFOC->radian) + pFOC->iqPID.out * cos_table(pFOC->radian);
}

/*************************************************************
** Function name:       CurrentPIControlID
** Descriptions:        D轴电流闭环
** Input parameters:    pFOC:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
static void CurrentPIControlID(PFOC_Struct pFOC)
{
    //获取实际值
    pFOC->idPID.pre = pFOC->id ;
    //获取目标值
    pFOC->idPID.tar = pFOC->tarid;
    //计算偏差
    pFOC->idPID.bias = pFOC->idPID.tar - pFOC->idPID.pre;
    //计算PID输出值
    pFOC->idPID.out += pFOC->idPID.kp * (pFOC->idPID.bias - pFOC->idPID.lastBias) + pFOC->idPID.ki * pFOC->idPID.bias;
    //保存偏差
    pFOC->idPID.lastBias = pFOC->idPID.bias;

		pFOC->idPID.out = LIMIT(pFOC->idPID.out,-pFOC->idPID.outMax,pFOC->idPID.outMax);

}
/*************************************************************
** Function name:       CurrentPIControlIQ
** Descriptions:        Q轴电流闭环
** Input parameters:    pFOC:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
static void CurrentPIControlIQ(PFOC_Struct pFOC)
{
    //获取实际值
    pFOC->iqPID.pre = pFOC->iq;
    //获取目标值
    pFOC->iqPID.tar = pFOC->tariq;
    //计算偏差
    pFOC->iqPID.bias = pFOC->iqPID.tar - pFOC->iqPID.pre;
    //计算PID输出值
    pFOC->iqPID.out += pFOC->iqPID.kp * (pFOC->iqPID.bias - pFOC->iqPID.lastBias) + pFOC->iqPID.ki * pFOC->iqPID.bias;
    //保存偏差
    pFOC->iqPID.lastBias = pFOC->iqPID.bias;
	
		pFOC->iqPID.out = LIMIT(pFOC->iqPID.out,-pFOC->iqPID.outMax,pFOC->iqPID.outMax);
}

/*************************************************************
** Function name:       FocContorl
** Descriptions:        FOC控制流程
** Input parameters:    pFOC:结构体指针
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void FocContorl(PFOC_Struct pFOC)
{
    //0.获取电气角度
		pFOC->radian +=200;
//    GetElectricalAngle(pFOC);
    //1计算实际电流值
   //1.0电流重构
		CurrentReconstruction(pFOC);
		//1.1Clarke变换
		ClarkeTransform(pFOC);
		//1.2Park变换
		ParkTransform(pFOC);
//    //2.做PID闭环
    CurrentPIControlID(pFOC);
    CurrentPIControlIQ(pFOC);
    //3.计算输出值iα i贝塔
    ParkAntiTransform(pFOC);
    //4.输出SVPWM
    pFOC->SvpwmGenerate(pFOC->ialphaSVPWM,pFOC->ibetaSVPWM);
}
/*************************************************************
** Function name:       SetCurrentPIDTar
** Descriptions:        设置D轴和Q轴的目标值
** Input parameters:    pFOC：FOC对象指针
**                      tarid：D轴目标电流
**                      tariq：Q轴目标电流
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void SetCurrentPIDTar(PFOC_Struct pFOC,float tarid,float tariq)
{
    pFOC->tarid = tarid;
    pFOC->tariq = tariq;
}
/*************************************************************
** Function name:       SetCurrentPIDParams
** Descriptions:        设置电流环参数
** Input parameters:    pFOC：FOC对象指针
**                      kp:比例
**                      ki:积分
**                      kd:微分
**                      outMax:输出限幅
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void SetCurrentPIDParams(PFOC_Struct pFOC,float kp,float ki,float kd,float outMax)
{
    pFOC->idPID.kp = kp;
    pFOC->idPID.ki = ki;
    pFOC->idPID.kd = kd;
    pFOC->idPID.outMax = outMax;

    pFOC->iqPID.kp = kp;
    pFOC->iqPID.ki = ki;
    pFOC->iqPID.kd = kd;
    pFOC->iqPID.outMax = outMax;
}
/*************************************************************
** Function name:       SetFocEnable
** Descriptions:        设置FOC使能
** Input parameters:    pFOC：FOC对象指针
**                      isEnable:是否使能
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void SetFocEnable(PFOC_Struct pFOC,uint8_t isEnable)
{
    pFOC->isEnable = isEnable;
		pFOC->SetEnable(pFOC->isEnable);
}


void SetTarIDIQ(float id,float iq)
{
	SetCurrentPIDTar(&gMotorFOC,id,iq);
}


#define ORDER 2
float b[ORDER + 1] = {0.20657208f, 0.41314417f, 0.20657208f};  // 从Python生成的b系数
float a[ORDER + 1] = {1.0f, -0.36952738f, 0.19581571f};        // 从Python生成的a系数

// 状态数组
float x[ORDER + 1] = {0};  // 输入历史值
float y[ORDER + 1] = {0};  // 输出历史值

float iir_filter(float input) {
    // 更新输入历史值
    for (int8_t i = ORDER; i > 0; --i) {
        x[i] = x[i - 1];
        y[i] = y[i - 1];
    }
    x[0] = input;
    
    // 计算当前输出
    float output = 0;
    for (int8_t i = 0; i <= ORDER; ++i) {
        output += b[i] * x[i];
    }
    for (int8_t i = 1; i <= ORDER; ++i) {
        output -= a[i] * y[i];
    }
    
    // 更新状态变量
    y[0] = output;
    
    return output;
}


static void GetMotorPreCurrent(float *ua,float *ub,float *uc)
{
		float value[3];
		value[0] = ((float)(lastAdcData[0]  + (ADC_offset[0][0] - ADC_offset[1][0]) )/2);
		value[1] = ((float)(lastAdcData[1]  + (ADC_offset[0][1] - ADC_offset[1][1]) )/2);
		value[2] = ((float)(lastAdcData[2]  + (ADC_offset[0][2] - ADC_offset[1][2]) )/2);
	
		*ua = value[0];
		FOCdata.adc[0] = value[0];
		
		*ub = value[1];
		FOCdata.adc[1] = value[1];
		
		*uc = value[2];
		FOCdata.adc[2] = value[2];
}

int16_t ADC_offset[2][RESIST_NUM];
void CalculateAdcOffset(void)
{
	SetTarIDIQ(0.0f,1.0f);
	SetCurrentPIDParams(&gMotorFOC,M_KP,M_KI,M_KD,M_OUTMAX);
	HAL_ADCEx_Calibration_Start(&ADC_PORT);    //ADC内部校准
	HAL_ADC_Start_DMA(&ADC_PORT, (uint32_t *)ADC_offset, RESIST_NUM);
}