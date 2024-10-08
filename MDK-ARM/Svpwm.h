#ifndef __SVPWM_H
#define __SVPWM_H

#include "FocCommon.h"

struct SSvpwm_Struct
{
    float u_alpha;		//阿尔法轴目标电压 	(最大值为母线电压 * sqrt(3) / 3)
    float u_beta;		//贝塔轴目标电压   	(最大值为母线电压 * sqrt(3) / 3)
    float u1;		//用于扇区判断
    float u2;		//用于扇区判断
    float u3;		//用于扇区判断
    float t0;		//0矢量作用时长
    float t1;		//1矢量作用时长
    float t2;		//2矢量作用时长
    float t3;		//3矢量作用时长
    float t4;		//4矢量作用时长
    float t5;		//5矢量作用时长
    float t6;		//6矢量作用时长
    float t7;		//7矢量作用时长
    int32_t ts;		//SVPWM周期
		int32_t maxTs;    //高电平最长时间
		int32_t adcTs;    //ADC稳定时间
    float udc;		//母线电压
    uint8_t sector;//扇区索引
    void (*SetChannelAHighLeaveTime_us)(int16_t time); //一个SVPWM周期内A相绕组高电平时间(中央对齐方式)
    void (*SetChannelBHighLeaveTime_us)(int16_t time); //一个SVPWM周期内B相绕组高电平时间(中央对齐方式)
    void (*SetChannelCHighLeaveTime_us)(int16_t time); //一个SVPWM周期内C相绕组高电平时间(中央对齐方式)
    void (*SetChannelDHighLeaveTime_us)(int16_t time); //一个SVPWM周期内C相绕组高电平时间(中央对齐方式)
};
typedef struct SSvpwm_Struct Svpwm_Struct;
typedef Svpwm_Struct *PSvpwm_Struct;


/*************************************************************
** Function name:       SVPWM_EXPORT
** Descriptions:        初始化一个SVPMW对象
** Input parameters:    x:对象名字
**                      xTs：SVPWM波形周期 单位us 建议 100
**                      xMaxTs：高电平最长时间 建议80 留20us给AD采样
**						xAdcTs：ADC稳定时间 建议1us
**                      xudc：母线电压 单位V 
**                      xSetChannelAHighLeaveTime_us：设置A相高电平时间
**                      xSetChannelBHighLeaveTime_us：设置B相高电平时间
**                      xSetChannelCHighLeaveTime_us：设置C相高电平时间
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
