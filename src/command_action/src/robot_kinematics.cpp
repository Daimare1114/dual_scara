#include "../include/command_action/robot_kinematics.h"
#include <math.h>
#include <stdio.h>

//Initialize Scara
int InitRBKinema(MotorParam *Param,int Mode){
    if (Mode == MODE_SCARA) {
        Param->CurAngle[INDEX_J1] = INIT_J1;
        Param->CurAngle[INDEX_J2] = INIT_J2;
        Param->CurAngle[INDEX_J3] = INIT_J3;

        CalcForwardKinematics(Param);
    }
    else if (Mode == MODE_6AXIS) {

    }
    else {
        return -1;
    }

    return 0;
}

//Forward Kinematics
void CalcForwardKinematics(MotorParam *Param) {
    float Theta1, Theta2;
    
    if (Param->CurAngle[INDEX_J2] > 0) {
        Param->ArmMode = SCARA_POSE_LEFT;
    }
    else {
        Param->ArmMode = SCARA_POSE_RIGHT;
    }

    if (fabs(Param->CurAngle[INDEX_J2]) < SINGULARITY_THRESH) {
        Param->SingularityState = SCARA_SINGULARITY;
    }
    else {
        Param->SingularityState = SCARA_NOT_SINGULARITY;
    }

    Theta1 = Param->CurAngle[INDEX_J1] * DEG2RAD;
    Theta2 = Param->CurAngle[INDEX_J2] * DEG2RAD;

    Param->Pos[INDEX_POS_X] = LINK1 * cos(Theta1) + LINK2 * cos(Theta1 + Theta2);
    Param->Pos[INDEX_POS_Y] = LINK1 * sin(Theta1) + LINK2 * sin(Theta1 + Theta2);
    //Param->Pos[INDEX_POS_Z] = 100;
}

//Inverse Kinematics
int CalcInverseKinematics(MotorParam *Param, int Mode) {
    float theta1, theta2;
    float PosX, PosY;
    float Tmp1, Tmp2;
    float Alpha;
    float Length;

    if (Mode == IK_MODE_DEST) {
        PosX = Param->DstPos[INDEX_POS_X];
        PosY = Param->DstPos[INDEX_POS_Y];
    }
    else if (Mode == IK_MODE_CURRENT) {
        PosX = Param->Pos[INDEX_POS_X];
        PosY = Param->Pos[INDEX_POS_Y];
    }
    else {}

    Length = pow(PosX, 2.0) + pow(PosY, 2.0);
    
    Tmp1 = Length - (pow(LINK1, 2.0) + pow(LINK2, 2.0));
    Tmp2 = 2.0 * LINK1 * LINK2;
    if (Param->ArmMode == SCARA_POSE_LEFT) {
        theta2 = acos(Tmp1 / Tmp2);
    }
    else if (Param->ArmMode == SCARA_POSE_RIGHT) {
        theta2 = -acos(Tmp1 / Tmp2);
    }else{}

    //printf("%lf\n", theta2 * RAD2DEG);
    if (fabs(theta2 * RAD2DEG) <= SINGULARITY_THRESH || (isnan(theta2) == true)) {
        return SCARA_SINGULARITY;
    }else{}

    Tmp1 = pow(LINK1, 2.0) + Length - pow(LINK2, 2.0);
    Tmp2 = 2.0 * LINK1 * sqrt(Length);
    Alpha = fabs(acos(Tmp1 / Tmp2));

    if (Param->ArmMode == SCARA_POSE_LEFT) {
        theta1 = atan2(PosY, PosX) - Alpha;
    }
    else if(Param->ArmMode == SCARA_POSE_RIGHT){
        theta1 = atan2(PosY, PosX) + Alpha;
    }
    else {}

    if (Mode == IK_MODE_DEST) {
        Param->DstAngle[INDEX_J1] = theta1 * RAD2DEG;
        Param->DstAngle[INDEX_J2] = theta2 * RAD2DEG;
    }
    else if (Mode == IK_MODE_CURRENT) {
        Param->CurAngle[INDEX_J1] = theta1 * RAD2DEG;
        Param->CurAngle[INDEX_J2] = theta2 * RAD2DEG;
    }
    else {}

    return SCARA_NOT_SINGULARITY;
}

int CalcHandSpeedToAngularVel(MotorParam *Param) {
    int Ret = SCARA_NOT_SINGULARITY;
    float InvJacobian[2][2];
    float AngularVel[2];

    Ret = CalcScaraJacobian(Param, InvJacobian, JACOBI_INV);

    if (Ret == SCARA_NOT_SINGULARITY) {
        AngularVel[INDEX_J1] = InvJacobian[0][0] * Param->DstVel[INDEX_POS_X] + InvJacobian[0][1] * Param->DstVel[INDEX_POS_Y];
        AngularVel[INDEX_J2] = InvJacobian[1][0] * Param->DstVel[INDEX_POS_X] + InvJacobian[1][1] * Param->DstVel[INDEX_POS_Y];

        Param->DstAngle[INDEX_J1] = Param->CurAngle[INDEX_J1] + (AngularVel[INDEX_J1] * RAD2DEG * SAMPLING_TIME);
        Param->DstAngle[INDEX_J2] = Param->CurAngle[INDEX_J2] + (AngularVel[INDEX_J2] * RAD2DEG * SAMPLING_TIME);
    }

    return Ret;
}

//Calc Jacbian
int CalcScaraJacobian(MotorParam *Param, float Jacobian[2][2], int Mode) {
    int Ret = SCARA_NOT_SINGULARITY;
    float Theta1, Theta2;
    float TmpJacobian[2][2];
    float Det;

    Theta1 = Param->CurAngle[INDEX_J1] * DEG2RAD;
    Theta2 = Param->CurAngle[INDEX_J2] * DEG2RAD;

    if (Mode == JACOBI_NORMAL) {
        Jacobian[0][0] = -LINK1 * sin(Theta1) - LINK2 * sin(Theta1 + Theta2);
        Jacobian[0][1] = -LINK2 * sin(Theta1 + Theta2);
        Jacobian[1][0] = LINK1 * cos(Theta1) + LINK2 * cos(Theta1 + Theta2);
        Jacobian[1][1] = LINK2 * cos(Theta1 + Theta2);
    }
    else if (Mode == JACOBI_INV) {
        TmpJacobian[0][0] = -LINK1 * sin(Theta1) - LINK2 * sin(Theta1 + Theta2);
        TmpJacobian[0][1] = -LINK2 * sin(Theta1 + Theta2);
        TmpJacobian[1][0] = LINK1 * cos(Theta1) + LINK2 * cos(Theta1 + Theta2);
        TmpJacobian[1][1] = LINK2 * cos(Theta1 + Theta2);

        Det = TmpJacobian[0][0] * TmpJacobian[1][1] - TmpJacobian[0][1] * TmpJacobian[1][0];
        //printf("%f\n", Det);
        if (fabs(Det) < 200) {
            printf("Singularity!\n");
            Ret = SCARA_SINGULARITY;
        }
        else {
            Jacobian[0][0] = TmpJacobian[1][1] / Det;
            Jacobian[0][1] = -TmpJacobian[0][1] / Det;
            Jacobian[1][0] = -TmpJacobian[1][0] / Det;
            Jacobian[1][1] = TmpJacobian[0][0] / Det;
        }
    }
    else {

    }

    return Ret;
}