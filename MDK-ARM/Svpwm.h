#ifndef __SVPWM_H
#define __SVPWM_H

#include "FocCommon.h"

struct SSvpwm_Struct
{
    float u_alpha;		//��������Ŀ���ѹ 	(���ֵΪĸ�ߵ�ѹ * sqrt(3) / 3)
    float u_beta;		//������Ŀ���ѹ   	(���ֵΪĸ�ߵ�ѹ * sqrt(3) / 3)
    float u1;		//���������ж�
    float u2;		//���������ж�
    float u3;		//���������ж�
    float t0;		//0ʸ������ʱ��
    float t1;		//1ʸ������ʱ��
    float t2;		//2ʸ������ʱ��
    float t3;		//3ʸ������ʱ��
    float t4;		//4ʸ������ʱ��
    float t5;		//5ʸ������ʱ��
    float t6;		//6ʸ������ʱ��
    float t7;		//7ʸ������ʱ��
    int32_t ts;		//SVPWM����
		int32_t maxTs;    //�ߵ�ƽ�ʱ��
		int32_t adcTs;    //ADC�ȶ�ʱ��
    float udc;		//ĸ�ߵ�ѹ
    uint8_t sector;//��������
    void (*SetChannelAHighLeaveTime_us)(int16_t time); //һ��SVPWM������A������ߵ�ƽʱ��(������뷽ʽ)
    void (*SetChannelBHighLeaveTime_us)(int16_t time); //һ��SVPWM������B������ߵ�ƽʱ��(������뷽ʽ)
    void (*SetChannelCHighLeaveTime_us)(int16_t time); //һ��SVPWM������C������ߵ�ƽʱ��(������뷽ʽ)
    void (*SetChannelDHighLeaveTime_us)(int16_t time); //һ��SVPWM������C������ߵ�ƽʱ��(������뷽ʽ)
};
typedef struct SSvpwm_Struct Svpwm_Struct;
typedef Svpwm_Struct *PSvpwm_Struct;


/*************************************************************
** Function name:       SVPWM_EXPORT
** Descriptions:        ��ʼ��һ��SVPMW����
** Input parameters:    x:��������
**                      xTs��SVPWM�������� ��λus ���� 100
**                      xMaxTs���ߵ�ƽ�ʱ�� ����80 ��20us��AD����
**						xAdcTs��ADC�ȶ�ʱ�� ����1us
**                      xudc��ĸ�ߵ�ѹ ��λV 
**                      xSetChannelAHighLeaveTime_us������A��ߵ�ƽʱ��
**                      xSetChannelBHighLeaveTime_us������B��ߵ�ƽʱ��
**                      xSetChannelCHighLeaveTime_us������C��ߵ�ƽʱ��
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
#define SVPWM_EXPORT(x,xTs,xMaxTs,xAdcTs,xudc,xSetChannelAHighLeaveTime_us,xSetChannelBHighLeaveTime_us,xSetChannelCHighLeaveTime_us,xSetChannelDHighLeaveTime_us)     \
Svpwm_Struct x = {                      \
    .u_alpha = 0,                       \
    .u_beta = 0,                        \
    .u1 = 0,                            \
    .u2 = 0,                            \
    .u3 = 0,                            \
    .t0 = 0,                            \
    .t1 = 0,                            \
    .t2 = 0,                            \
    .t3 = 0,                            \
    .t4 = 0,                            \
    .t5 = 0,                            \
    .t6 = 0,                            \
    .t7 = 0,                            \
    .ts = xTs,                          \
		.maxTs = xMaxTs,                    \
		.adcTs = xAdcTs,                    \
    .udc = xudc,                        \
    .sector = 0,                        \
    .SetChannelAHighLeaveTime_us = xSetChannelAHighLeaveTime_us,            \
    .SetChannelBHighLeaveTime_us = xSetChannelBHighLeaveTime_us,            \
    .SetChannelCHighLeaveTime_us = xSetChannelCHighLeaveTime_us,            \
    .SetChannelDHighLeaveTime_us = xSetChannelDHighLeaveTime_us,            \
};



void SvpwmContorol(float u_alpha,float u_beta);
uint8_t GetSVPWMSector(void);


extern Svpwm_Struct gMotor;

#endif /* Svpwm_h */
