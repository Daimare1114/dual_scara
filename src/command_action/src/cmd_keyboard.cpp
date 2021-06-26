#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include "robot_state_msg/RobotState.h"
#include "../include/command_action/motion_planning.h"
#include "../include/command_action/cmd_keyboard.h"

static void chatterCallback(const robot_state_msg::RobotState& msg);

int CmdState[2];

PranningParam R_PParam;//For Right motion plannig paramer
PranningParam L_PParam;//For Left motion plannig paramer
MotorParam R_MParam;//For Right motor parameter
MotorParam L_MParam;//For Left motor parameter

int main(int argc, char** argv){
    int Ret = NO_ERROR;
    int i = 0;

    CmdState[0] = CMD_NONE;
    CmdState[1] = CMD_NONE;

    //initialize parameter
    InitPlannnigState(&R_PParam);
    InitPlannnigState(&L_PParam);
    InitRBKinema(&R_MParam, MODE_SCARA);
    InitRBKinema(&L_MParam, MODE_SCARA);

    //set ros node
    ros::init(argc, argv, "command_keyboard");
    ros::NodeHandle nh;

    //set ros publisher topic
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("robot_angle", 2);
    std_msgs::Float32MultiArray pub_msg;
    pub_msg.data.resize(ANGLE_NUM);

    //set ros subscriver topic
    ros::Subscriber sub = nh.subscribe("robot_state", 2, chatterCallback);

    //set loop rate
    ros::Rate loop_rate(LOOP_RATE);//[Hz]

    /***********/
    /*Main Loop*/
    /***********/
    while(ros::ok()){
        
        //Check command
        ros::spinOnce();
        if(CmdState[0] == CMD_NONE && CmdState[1] == CMD_NONE){
            printf("input command\n");
            printf("L, R->");
            scanf("%d, %d", &CmdState[0], &CmdState[1]);
        }
        
        //Left Arm
        switch (CmdState[0])
        {
        case CMD_NONE:
            break;
        case CMD_JOINT_MANUAL:
            if (L_PParam.PlanningState == STATE_STANDBY) {
				printf("Current Joint\n");
				printf("[J1, J2] = [%f, %f]\n", L_MParam.CurAngle[0], L_MParam.CurAngle[1]);
				printf("Input destination angle\n");
				printf("J1, J2, vel_ratio = ");
				scanf("%f, %f, %f", &L_MParam.DstAngle[0], &L_MParam.DstAngle[1], &L_MParam.VelRatio);
				L_PParam.PlanningState = STATE_PLANNING;
			}
			else if (L_PParam.PlanningState == STATE_PLANNING) {
				printf("start planning\n");
                ExecMoveJoint(&L_PParam, &L_MParam, L_MParam.VelRatio);
			}
			else if (L_PParam.PlanningState == STATE_EXECUTE) {
				ExecMoveJoint(&L_PParam, &L_MParam, L_MParam.VelRatio);
				pub_msg.data[INDEX_J1] = L_MParam.DstAngle[INDEX_J1];
                pub_msg.data[INDEX_J2] = L_MParam.DstAngle[INDEX_J2];

				//printf("[J1, J2] = [%f, %f]\n", L_MParam.DstAngle[INDEX_J1], L_MParam.DstAngle[INDEX_J2]);
				pub.publish(pub_msg);
			}
			else if (L_PParam.PlanningState == STATE_FINISH) {
				L_PParam.PlanningState = STATE_STANDBY;
				CmdState[0] = CMD_NONE;
			}
			else {}
            break;
        default:
            break;
        }

        //Right Arm
        switch (CmdState[1])
        {
        case CMD_NONE:
            break;
        case CMD_JOINT_MANUAL:
            if (R_PParam.PlanningState == STATE_STANDBY) {
				printf("Current Joint\n");
				printf("[J1, J2] = [%f, %f]\n", R_MParam.CurAngle[0], R_MParam.CurAngle[1]);
				printf("Input destination angle\n");
				printf("J1, J2, vel_ratio = \n");
				scanf("%f, %f, %f", &R_MParam.DstAngle[0], &R_MParam.DstAngle[1], &R_MParam.VelRatio);
				R_PParam.PlanningState = STATE_PLANNING;
			}
			else if (R_PParam.PlanningState == STATE_PLANNING) {
				ExecMoveJoint(&R_PParam, &R_MParam, R_MParam.VelRatio);
			}
			else if (R_PParam.PlanningState == STATE_EXECUTE) {
				ExecMoveJoint(&R_PParam, &R_MParam, R_MParam.VelRatio);
				//for (i = 0; i < ANGLE_NUM; i++) {
					//pub_msg.data[i] = R_MParam.DstAngle[i];
				//}
				//printf("[J1, J2] = [%f, %f]\n", MParam->CurAngle[0], MParam->CurAngle[1]);
				//pub.publish(pub_msg);
			}
			else if (R_PParam.PlanningState == STATE_FINISH) {
				R_PParam.PlanningState = STATE_STANDBY;
				CmdState[1] = CMD_NONE;
			}
			else {}
            break;
        default:
            break;
        }

        loop_rate.sleep();
    }

    return Ret;
}

static void chatterCallback(const robot_state_msg::RobotState& msg){
    printf("subscribe! J1: %f\n", msg.cur_angle[INDEX_J1]);
    //Left hand state
    //angle
    L_MParam.CurAngle[INDEX_J1] = msg.cur_angle[INDEX_J1];
    L_MParam.CurAngle[INDEX_J2] = msg.cur_angle[INDEX_J2];
    
    //end effector pose
    L_MParam.Pos[INDEX_POS_X] = msg.pos_x;
    L_MParam.Pos[INDEX_POS_Y] = msg.pos_y;
    L_MParam.Pos[INDEX_POS_Z] = msg.pos_z;
    
    //arm mode
    L_MParam.ArmMode = msg.arm_mode;
    
    //gripp state
    L_MParam.GrippState = msg.gripp_state;

    //Right hand state
    //angle

    //end effector pose

    //arm mode

    //gripp state
}
