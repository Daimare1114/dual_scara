#include "robot_kinematics.h"

#define NO_ERROR		0

#define STATE_STANDBY	0//実行待ち
#define STATE_PLANNING	1//軌道計画中
#define STATE_EXECUTE	2//起動生成中
#define STATE_FINISH	3//軌道生成修了
#define STATE_ERROR		-1//存在しない状態

//ACC+CONST+DCC = 1.0になること
#define ACC_RATIO		(float)(0.25)
#define CONST_RATIO		(float)(0.5)
#define	DCC_RATIO		(float)(0.25)

typedef  struct {
    int		PlanningState;
    float	DstTime;
    float	AccTime;
    float	ConstTime;
    float	DccTime;
    float	ElappsedTime;
    float	DstPos[DIM3];
    float	DstAngle[ANGLE_NUM];
    float	AccCoef[ANGLE_NUM][6];
    float	ConstCoef[ANGLE_NUM][6];
    float	DccCoef[ANGLE_NUM][6];
}PranningParam;

void InitPlannnigState(PranningParam *Param);
int ExecMoveJoint(PranningParam *PParam, MotorParam *MParam, float VelRatio);
int ExecMovePosPtP(PranningParam *PParam, MotorParam *MParam, float VelRatio);
int ExecMovePosStraight(PranningParam *PParam, MotorParam *MParam, float VelRatio);