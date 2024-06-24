#ifndef FOC_h
#define FOC_h

#include "FocCommon.h"


extern volatile uint32_t gFoc_TimeCNT;

/*
增量式PI调节
*/
struct PI_Struct
{
    /* data */
    float kp;
    float ki;
    float kd;

    float pre;
    float tar;
    float bias;
    float lastBias;
    float out;
    float outMax;
};


/*
FOC结构体
*/
struct SFOC_Struct
{
    uint8_t isEnable;   //电机使能标志
    uint8_t polePairs;  //极对数
	
	  uint8_t iNum;         //a 电流采样个数
    float ia;           //a 相实际电流
    float ib;           //b 相实际电流
		float ic;           //b 相实际电流

    float tariq;        //目标电流
    float tarid;        //目标电流
    float angle;        //角度
    int16_t radian;       //弧度

    float ialpha;           //clarke变换后 * 2/3后 Iα
    float ibeta;           //clarke变换后 * 2/3后 Iβ
    float iq;           //park变换后 Iq
    float id;           //park变换后 Id

    float ialphaSVPWM;      //park反变换后 后 Iα
    float ibetaSVPWM;      //park反变换后 后 Iβ

    struct PI_Struct idPID;
    struct PI_Struct iqPID;
    void (*SetEnable)(uint8_t isEnable);
    float (*GetEncoderAngle)(void);
    uint8_t (*GetSVPWMSector)(void);
    void (*GetPreCurrent)(float *ua,float *ub,float *uc);
    void (*SvpwmGenerate)(float u_alpha,float u_beta);
};
typedef struct SFOC_Struct FOC_Struct;
typedef FOC_Struct* PFOC_Struct;

/*************************************************************
** Function name:       FOC_EXPORT
** Descriptions:        声明一个FOC对象
**                      xGetPreCurrent：获取相电流值，注意检测时间窗口
**						xiNum：电流采集相数（主要用来区分是否为3相电流）
**                      xSvpwmGenerate：SVPWM生成函数。配合SVPWM.h
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
#define FOC_EXPORT(x,xpolePairs,xiNum,xSetEnable,xGetEncoderAngle,xGetSVPWMSector,xGetPreCurrent,xSvpwmGenerate)    \
FOC_Struct x = {                            \
    .isEnable = 0,                          \
    .polePairs = xpolePairs,                \
    .tarid = 0.0,                           \
    .tariq = 0.0,                           \
    .angle = 0.0,                           \
    .radian = 0.0,                          \
    .iNum = xiNum,                          \
    .ia = 0,                              	\
    .ib = 0,                              	\
    .ic = 0,                              	\
    .ialpha = 0.0,                          \
    .ibeta = 0.0,                           \
    .iq = 0.0,                              \
    .id = 0.0,                              \
    .ialphaSVPWM = 0.0,                     \
    .ibetaSVPWM = 0.0,                      \
    .idPID = {0},                           \
    .iqPID = {0},                           \
    .SetEnable = xSetEnable,          			\
    .GetEncoderAngle = xGetEncoderAngle,    \
    .GetSVPWMSector = xGetSVPWMSector,      \
    .GetPreCurrent = xGetPreCurrent,        \
    .SvpwmGenerate = xSvpwmGenerate,        \
};


void FocContorl(PFOC_Struct pFOC);
void SetCurrentPIDTar(PFOC_Struct pFOC,float tarid,float tariq);
void SetCurrentPIDParams(PFOC_Struct pFOC,float kp,float ki,float kd,float outMax);
void SetFocEnable(PFOC_Struct pFOC,uint8_t isEnable);
void CalculateAdcOffset(void);
void CurrentReconstruction(PFOC_Struct pFOC);
void ClarkeTransform(PFOC_Struct pFOC);
void SetTarIDIQ(float id,float iq);

extern FOC_Struct gMotorFOC;
extern ADC_HandleTypeDef ADC_PORT;
extern int16_t ADC_offset[2][RESIST_NUM];
extern int16_t *ADC_Point;

#endif /* FOC_h */
