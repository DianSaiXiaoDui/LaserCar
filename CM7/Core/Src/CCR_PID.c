#include "CCR_PID.h"
#include <stdint.h>
#include <math.h>

CCR_PID_Struct CCRX_PID;
CCR_PID_Struct CCRY_PID;


void CCRX_PID_Init(float Kp,float Ki,float Kd,float P,float I, float D, float IThresh, float CurrentError, float LastError, float ErrorThresh, float ErrorInt,
		float Current, float Target,int32_t DeltaCCR, int32_t OutputThreshH,int32_t OutputThreshL,uint32_t Reset)
{
	CCRX_PID.Kp = Kp;               // 比例系数
	CCRX_PID.Ki = Ki;               // 积分系数
	CCRX_PID.Kd = Kd;               // 微分系数
	CCRX_PID.P  = P;                // 比例项
	CCRX_PID.I  = I;                // 积分项
	CCRX_PID.D  = D;                // 微分项
	CCRX_PID.IThresh = IThresh;     // 积分限幅
	CCRX_PID.CurrentError  = CurrentError;           // 当前误差
	CCRX_PID.LastError   = LastError;           // 上一误差
	CCRX_PID.ErrorThresh = ErrorThresh;  // 抗积分饱和临界误差
	CCRX_PID.ErrorInt    = ErrorInt;     // 累计误差
	CCRX_PID.Current     = Current;      // 当前横/纵坐标
	CCRX_PID.Target      = Target;          // 目标横/纵坐标
	CCRX_PID.DeltaCCR    = DeltaCCR;        // pid输出量：电机CCR改变值
	CCRX_PID.OutputThreshH = OutputThreshH; // pid输出限幅（上界）
	CCRX_PID.OutputThreshL = OutputThreshL; // pid输出限幅（下界）
	CCRX_PID.Reset         = Reset;        	// 切换目标标志
}

void CCRY_PID_Init(float Kp,float Ki,float Kd,float P,float I, float D, float IThresh, float CurrentError, float LastError, float ErrorThresh, float ErrorInt,
		float Current, float Target,int32_t DeltaCCR, int32_t OutputThreshH,int32_t OutputThreshL,uint32_t Reset)
{
	CCRY_PID.Kp = Kp;               // 比例系数
	CCRY_PID.Ki = Ki;               // 积分系数
	CCRY_PID.Kd = Kd;               // 微分系数
	CCRY_PID.P  = P;                // 比例项
	CCRY_PID.I  = I;                // 积分项
	CCRY_PID.D  = D;                // 微分项
	CCRY_PID.IThresh = IThresh;     // 积分限幅
	CCRY_PID.CurrentError  = CurrentError;      // 当前误差
	CCRY_PID.LastError   = LastError;           // 上一误差
	CCRY_PID.ErrorThresh = ErrorThresh;  // 抗积分饱和临界误差
	CCRY_PID.ErrorInt    = ErrorInt;     // 累计误差
	CCRY_PID.Current     = Current;      // 当前横/纵坐标
	CCRY_PID.Target      = Target;          // 目标横/纵坐标
	CCRY_PID.DeltaCCR    = DeltaCCR;        // pid输出量：电机CCR改变值
	CCRY_PID.OutputThreshH = OutputThreshH; // pid输出限幅（上界）
	CCRY_PID.OutputThreshL = OutputThreshL; // pid输出限幅（下界）
	CCRY_PID.Reset         = Reset;        	// 切换目标标志
}

//全量式pid ,更新pid输出
void CCRX_PID_Update(void)
{
	CCRX_PID.CurrentError = CCRX_PID.Target - CCRX_PID.Current;
	if (CCRX_PID.Reset)
	{
		CCRX_PID.LastError = CCRX_PID.CurrentError;
		CCRX_PID.Reset = 0;
	}
	CCRX_PID.P = CCRX_PID.Kp * CCRX_PID.CurrentError;	// 计算P项
	if (fabs(CCRX_PID.CurrentError) > CCRX_PID.ErrorThresh)	//计算I项
	{
		CCRX_PID.ErrorInt = 0;
	}
	else
	{
		CCRX_PID.ErrorInt += CCRX_PID.CurrentError;  // 累计误差积分
	}
	CCRX_PID.I = CCRX_PID.ErrorInt * CCRX_PID.Ki;  // I项=累计误差积分*积分系数
	// 积分限幅
    CCRX_PID.I = (CCRX_PID.I > CCRX_PID.IThresh)? CCRX_PID.IThresh:CCRX_PID.I;
	CCRX_PID.I = (CCRX_PID.I < -CCRX_PID.IThresh)? -CCRX_PID.IThresh:CCRX_PID.I;

	CCRX_PID.D = CCRX_PID.Kd * (CCRX_PID.CurrentError - CCRX_PID.LastError);	// 计算D项

	CCRX_PID.DeltaCCR = (int32_t)(CCRX_PID.P + CCRX_PID.I + CCRX_PID.D);// PID输出=P项+I项+D项
	// 输出限幅
	CCRX_PID.DeltaCCR = (CCRX_PID.DeltaCCR > CCRX_PID.OutputThreshH)? CCRX_PID.OutputThreshH:CCRX_PID.DeltaCCR;
	CCRX_PID.DeltaCCR = (CCRX_PID.DeltaCCR < CCRX_PID.OutputThreshL)? CCRX_PID.OutputThreshL:CCRX_PID.DeltaCCR;

	CCRX_PID.LastError = CCRX_PID.CurrentError;// 更新上一次误差
}

void CCRY_PID_Update(void)
{
	CCRY_PID.CurrentError = CCRY_PID.Target - CCRY_PID.Current;
	if (CCRY_PID.Reset)
	{
		CCRY_PID.LastError = CCRY_PID.CurrentError;
		CCRY_PID.Reset = 0;
	}
	CCRY_PID.P = CCRY_PID.Kp * CCRY_PID.CurrentError;	// 计算P项
	if (fabs(CCRY_PID.CurrentError) > CCRY_PID.ErrorThresh)	//计算I项
	{
		CCRY_PID.ErrorInt = 0;
	}
	else
	{
		CCRY_PID.ErrorInt += CCRY_PID.CurrentError;  // 累计误差积分
	}
	CCRY_PID.I = CCRY_PID.ErrorInt * CCRY_PID.Ki;  // I项=累计误差积分*积分系数
	// 积分限幅
    CCRY_PID.I = (CCRY_PID.I > CCRY_PID.IThresh)? CCRY_PID.IThresh:CCRY_PID.I;
	CCRY_PID.I = (CCRY_PID.I < -CCRY_PID.IThresh)? -CCRY_PID.IThresh:CCRY_PID.I;

	CCRY_PID.D = CCRY_PID.Kd * (CCRY_PID.CurrentError - CCRY_PID.LastError);	// 计算D项

	CCRY_PID.DeltaCCR = (int32_t)(CCRY_PID.P + CCRY_PID.I + CCRY_PID.D);// PID输出=P项+I项+D项
	// 输出限幅
	CCRY_PID.DeltaCCR = (CCRY_PID.DeltaCCR > CCRY_PID.OutputThreshH)? CCRY_PID.OutputThreshH:CCRY_PID.DeltaCCR;
	CCRY_PID.DeltaCCR = (CCRY_PID.DeltaCCR < CCRY_PID.OutputThreshL)? CCRY_PID.OutputThreshL:CCRY_PID.DeltaCCR;

	CCRY_PID.LastError = CCRY_PID.CurrentError;// 更新上一次误差
}


void CCRX_PID_Reset(void)
{
	CCRX_PID.ErrorInt = 0;
	CCRX_PID.Reset = 1;
}

void CCRY_PID_Reset(void)
{
	CCRY_PID.ErrorInt = 0;
	CCRY_PID.Reset = 1;
}


