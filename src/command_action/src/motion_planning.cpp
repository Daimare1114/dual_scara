#include "../include/command_action/motion_planning.h"
#include <stdio.h>
#include <math.h>

static void CalcTrajectoryCoef(float *AccCoef, float *ConstCoef, float *DccCoef, 
                               float AccTime, float ConstTime, float DccTime,
                               float DstTime, float DstVal, float CurVal);
static float CalcCmdTrajectory(float *CmdCoef, float ElaspedTime, float ConstTime);

void InitPlannnigState(PranningParam *Param){
    int i = 0;
    int j = 0;

    Param->AccTime = 0;
    Param->ConstTime = 0;
    Param->DccTime = 0;
    Param->DstTime = 0;
    Param->ElappsedTime = 0;
    for(i = 0; i < ANGLE_NUM; i++){
        Param->DstAngle[i] = 0;
        for(j = 0; j < 6; j++){
            Param->AccCoef[i][j] = 0;
            Param->ConstCoef[i][j] = 0;
            Param->DccCoef[i][j] = 0;
        }
    }
    Param->PlanningState = STATE_STANDBY;
}

int ExecMoveJoint(PranningParam *PParam, MotorParam *MParam, float VelRatio) {
    int i = 0;
    int Ret = NO_ERROR;
    float TmpDst;

    switch (PParam->PlanningState) {
    case STATE_STANDBY:
        break;
    case STATE_PLANNING:
        //calc destination time about each angle
        PParam[0].DstTime = 0;
        for (i = 0; i < ANGLE_NUM; i++) {
            PParam->DstAngle[i] = MParam->DstAngle[i];
            TmpDst = fabs(PParam->DstAngle[i] - MParam->CurAngle[i]);
            if (TmpDst > PParam->DstTime) {
                PParam->DstTime = TmpDst;
            }
        }
        //deside destination time
        PParam->DstTime = (2 * PParam->DstTime) / ((CONST_RATIO + 1) * (MOTOR_MAXSPEED * VelRatio));
        printf("Destination time: %f[s]\n", PParam->DstTime);
        //calc each part time
        PParam->AccTime = ACC_RATIO * PParam->DstTime;
        PParam->ConstTime = CONST_RATIO * PParam->DstTime;
        PParam->DccTime = DCC_RATIO * PParam->DstTime;


        //calc coeficient
        for (i = 0; i < ANGLE_NUM; i++) {
            CalcTrajectoryCoef(PParam->AccCoef[i], PParam->ConstCoef[i], PParam->DccCoef[i],
                PParam->AccTime, PParam->ConstTime, PParam->DccTime,
                PParam->DstTime, PParam->DstAngle[i], MParam->CurAngle[i]);
        }

        //update state of motion planning
        PParam->PlanningState = STATE_EXECUTE;
        PParam->ElappsedTime = 0.0;
        
        break;
    case STATE_EXECUTE:
        //calc desired angle for nextstep
        if (PParam[0].ElappsedTime <= PParam[0].AccTime) {
            //calc desired angle of acceleration section
            for (i = 0; i < ANGLE_NUM; i++) {
                MParam->DstAngle[i] = CalcCmdTrajectory(PParam->AccCoef[i], PParam->ElappsedTime, 0.0);
            }
        }
        else if ((PParam->ElappsedTime > PParam->AccTime)
            && (PParam->ElappsedTime <= (PParam->AccTime + PParam->ConstTime))) {
            //calc desired angle of constant velocity section
            for (i = 0; i < ANGLE_NUM; i++) {
                MParam->DstAngle[i] = CalcCmdTrajectory(PParam->ConstCoef[i], PParam->ElappsedTime, PParam->AccTime);
            }
        }
        else if ((PParam->ElappsedTime > (PParam->AccTime + PParam->ConstTime))
            && (PParam->ElappsedTime <= PParam->DstTime)) {
            //calc desired angle of deceleration section
            for (i = 0; i < ANGLE_NUM; i++) {
                MParam->DstAngle[i] = CalcCmdTrajectory(PParam->DccCoef[i], PParam->ElappsedTime, (PParam->AccTime + PParam->ConstTime));
            }
        }
        else {
            //update state
            PParam->PlanningState = STATE_FINISH;
        }

        //update sampling time
        PParam->ElappsedTime = PParam->ElappsedTime + SAMPLING_TIME;

        break;
    case STATE_FINISH:
        break;
    default:
        Ret = STATE_ERROR;
        break;
    }

    return Ret;
}
	
int ExecMovePosPtP(PranningParam *PParam, MotorParam *MParam, float VelRatio) {
    int i = 0;
    int Ret = NO_ERROR;
    float TmpDst;

    switch (PParam->PlanningState) {
    case STATE_STANDBY:
        break;
    case STATE_PLANNING:
   
        Ret = CalcInverseKinematics(MParam, IK_MODE_DEST);
        if (Ret != SCARA_SINGULARITY) {
            PParam[0].DstTime = 0;
            for (i = 0; i < ANGLE_NUM; i++) {
                PParam->DstAngle[i] = MParam->DstAngle[i];
                TmpDst = fabs(PParam->DstAngle[i] - MParam->CurAngle[i]);
                if (TmpDst > PParam->DstTime) {
                    PParam->DstTime = TmpDst;
                }
            }
            
            PParam->DstTime = (2 * PParam->DstTime) / ((CONST_RATIO + 1) * (MOTOR_MAXSPEED * VelRatio));
            printf("Destination time: %f[s]\n", PParam->DstTime);
            
            PParam->AccTime = ACC_RATIO * PParam->DstTime;
            PParam->ConstTime = CONST_RATIO * PParam->DstTime;
            PParam->DccTime = DCC_RATIO * PParam->DstTime;


            for (i = 0; i < ANGLE_NUM; i++) {
                CalcTrajectoryCoef(PParam->AccCoef[i], PParam->ConstCoef[i], PParam->DccCoef[i],
                                    PParam->AccTime, PParam->ConstTime, PParam->DccTime,
                                    PParam->DstTime, PParam->DstAngle[i], MParam->CurAngle[i]);
            }

            
            PParam->PlanningState = STATE_EXECUTE;
            PParam->ElappsedTime = 0.0;
        }
        else {
            printf("Singularity\n");
            PParam->PlanningState = STATE_FINISH;
        }

        break;
    case STATE_EXECUTE:
        
        if (PParam[0].ElappsedTime <= PParam[0].AccTime) {
            
            for (i = 0; i < ANGLE_NUM; i++) {
                MParam->DstAngle[i] = CalcCmdTrajectory(PParam->AccCoef[i], PParam->ElappsedTime, 0.0);
            }
        }
        else if ((PParam->ElappsedTime > PParam->AccTime)
            && (PParam->ElappsedTime <= (PParam->AccTime + PParam->ConstTime))) {
            
            for (i = 0; i < ANGLE_NUM; i++) {
                MParam->DstAngle[i] = CalcCmdTrajectory(PParam->ConstCoef[i], PParam->ElappsedTime, PParam->AccTime);
            }
        }
        else if ((PParam->ElappsedTime > (PParam->AccTime + PParam->ConstTime))
            && (PParam->ElappsedTime <= PParam->DstTime)) {
            
            for (i = 0; i < ANGLE_NUM; i++) {
                MParam->DstAngle[i] = CalcCmdTrajectory(PParam->DccCoef[i], PParam->ElappsedTime, (PParam->AccTime + PParam->ConstTime));
            }
        }
        else {
            
            PParam->PlanningState = STATE_FINISH;
        }

        
        PParam->ElappsedTime = PParam->ElappsedTime + SAMPLING_TIME;

        break;
    case STATE_FINISH:
        break;
    default:
        Ret = STATE_ERROR;
        break;
    }
    
    return Ret;
}


int ExecMovePosStraight(PranningParam *PParam, MotorParam *MParam, float VelRatio) {
    int Ret = NO_ERROR;
    int i = 0;
    float TmpDst;

    switch (PParam[0].PlanningState) {
    case STATE_STANDBY:
        break;
    case STATE_PLANNING:
        PParam[0].DstTime = 0;
        for (i = 0; i < DIM3; i++) {
            PParam->DstPos[i] = MParam->DstPos[i];
            TmpDst = fabs(PParam->DstPos[i] - MParam->Pos[i]);
            if (TmpDst > PParam->DstTime) {
                PParam->DstTime = TmpDst;
            }
        }
        PParam->DstTime = (2 * PParam->DstTime) / ((CONST_RATIO + 1) * (HAND_MAXSPEED * VelRatio));
        //printf("Destination time: %f[s]\n", PParam->DstTime);
   
        PParam->AccTime = ACC_RATIO * PParam->DstTime;
        PParam->ConstTime = CONST_RATIO * PParam->DstTime;
        PParam->DccTime = DCC_RATIO * PParam->DstTime;

        for (i = 0; i < DIM3; i++) {
            CalcTrajectoryCoef(PParam->AccCoef[i], PParam->ConstCoef[i], PParam->DccCoef[i],
                                PParam->AccTime, PParam->ConstTime, PParam->DccTime,
                                PParam->DstTime, PParam->DstPos[i], MParam->Pos[i]);
        }

        PParam->PlanningState = STATE_EXECUTE;
        PParam->ElappsedTime = 0.0;

        break;
    case STATE_EXECUTE:
       
        if (PParam[0].ElappsedTime <= PParam[0].AccTime) {
            
            for (i = 0; i < DIM3; i++) {
                MParam->DstPos[i] = CalcCmdTrajectory(PParam->AccCoef[i], PParam->ElappsedTime, 0.0);
            }
        }
        else if ((PParam->ElappsedTime > PParam->AccTime)
                && (PParam->ElappsedTime <= (PParam->AccTime + PParam->ConstTime))) {
            
            for (i = 0; i < DIM3; i++) {
                MParam->DstPos[i] = CalcCmdTrajectory(PParam->ConstCoef[i], PParam->ElappsedTime, PParam->AccTime);
            }
        }
        else if ((PParam->ElappsedTime > (PParam->AccTime + PParam->ConstTime))
                && (PParam->ElappsedTime <= PParam->DstTime)) {
            
            for (i = 0; i < DIM3; i++) {
                MParam->DstPos[i] = CalcCmdTrajectory(PParam->DccCoef[i], PParam->ElappsedTime, (PParam->AccTime + PParam->ConstTime));
            }
        }
        else {
            
            PParam->PlanningState = STATE_FINISH;
        }

        
        Ret = CalcInverseKinematics(MParam, IK_MODE_DEST);

        
        PParam->ElappsedTime = PParam->ElappsedTime + SAMPLING_TIME;

        break;
    case STATE_FINISH:
        break;
    default:
        Ret = STATE_ERROR;
        break;
    }

    return Ret;
}

static void CalcTrajectoryCoef(float *AccCoef, float *ConstCoef, float *DccCoef, 
                               float AccTime, float ConstTime, float DccTime,
                               float DstTime, float DstVal, float CurVal) {
    
    float TmpTilt, TmpIntrcpt;
    float TmpPos, TmpVel;

  
    TmpTilt = (DstVal - CurVal) / ((DstTime - (DccTime / 2)) - (AccTime / 2));
    TmpIntrcpt = CurVal - (TmpTilt * (AccTime / 2));
    ConstCoef[0] = TmpTilt * AccTime + TmpIntrcpt;
    ConstCoef[1] = TmpTilt;
    ConstCoef[2] = 0.0;
    ConstCoef[3] = 0.0;
    ConstCoef[4] = 0.0;
    ConstCoef[5] = 0.0;

    
    TmpPos = TmpTilt * AccTime + TmpIntrcpt;;
    TmpVel = (2 * (TmpPos - CurVal)) / AccTime;
    AccCoef[0] = CurVal;
    AccCoef[1] = 0.0;
    AccCoef[2] = 0.0;
    AccCoef[3] = (1 / (2 * powf(AccTime, 3.0))) * ((20 * TmpPos - 20 * CurVal) - (8 * TmpVel) * AccTime);
    AccCoef[4] = (1 / (2 * powf(AccTime, 4.0))) * ((30 * CurVal - 30 * TmpPos) + (14 * TmpVel) * AccTime);
    AccCoef[5] = 0.0;

   
    TmpPos = TmpTilt * (AccTime + ConstTime) + TmpIntrcpt;
    TmpVel = (2 * (DstVal - TmpPos)) / DccTime;
    DccCoef[0] = TmpPos;
    DccCoef[1] = TmpVel;
    DccCoef[2] = 0.0;
    DccCoef[3] = (1 / (2 * powf(DccTime, 3.0))) * ((20 * DstVal - 20 * TmpPos) - (12 * TmpVel) * DccTime);
    DccCoef[4] = (1 / (2 * powf(DccTime, 4.0))) * ((30 * TmpPos - 30 * DstVal) + (16 * TmpVel) * DccTime);
    DccCoef[5] = 0.0;

}

static float CalcCmdTrajectory(float *CmdCoef, float ElaspedTime, float ConstTime) {
    float Cmd = 0.0;
    float Time = ElaspedTime - ConstTime;

    Cmd = CmdCoef[0]
        + CmdCoef[1] * Time
        + CmdCoef[2] * powf(Time, 2.0)
        + CmdCoef[3] * powf(Time, 3.0)
        + CmdCoef[4] * powf(Time, 4.0);

    return Cmd;
}