#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include "robot_state_msg/RobotState.h"
#include "../include/command_action/motion_planning.h"
#include "../include/command_action/cmd_controller.h"

static void joy_callback(const sensor_msgs::Joy &joy_msg);
static void chatterCallback(const robot_state_msg::RobotState& msg);
static void chatterCallback2(const robot_state_msg::RobotState& msg);

PranningParam R_PParam;//For Right motion plannig paramer
PranningParam L_PParam;//For Left motion plannig paramer
MotorParam R_MParam;//For Right motor parameter
MotorParam L_MParam;//For Left motor parameter
CommandState R_CmdState;
CommandState L_CmdState;

int main(int argc, char** argv){

    int Ret = NO_ERROR;
    int i = 0;
    float L_TmpAngle[2];
    float L_OldAngle[2];
    float R_TmpAngle[2];
    float R_OldAngle[2];

    //initialize parameter
    InitPlannnigState(&R_PParam);
    InitPlannnigState(&L_PParam);
    InitRBKinema(&R_MParam, MODE_SCARA);
    InitRBKinema(&L_MParam, MODE_SCARA);
    R_CmdState.Action = CMD_NONE;
    L_CmdState.Action = CMD_NONE;
    L_OldAngle[INDEX_J1] = INIT_J1;
    L_OldAngle[INDEX_J2] = INIT_J2;
    R_OldAngle[INDEX_J1] = -INIT_J1;
    R_OldAngle[INDEX_J2] = -INIT_J2;

    ros::init(argc, argv, "command_controller");
    ros::NodeHandle nh;

    //set ros publisher topic
    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("robot_angle", 2);
    std_msgs::Float32MultiArray pub_msg;
    pub_msg.data.resize(ANGLE_NUM);

    //set ros subscriver topic
    ros::Subscriber sub = nh.subscribe("robot_state", 2, chatterCallback);
    ros::Subscriber sub2 = nh.subscribe("robot_state2", 2, chatterCallback2);
    ros::Subscriber sub_joy = nh.subscribe("joy", 10, joy_callback);

    //set loop rate
    ros::Rate loop_rate(LOOP_RATE);//[Hz]

    /***********/
    /*Main Loop*/
    /***********/
    while(ros::ok()){
        L_TmpAngle[INDEX_J1] = L_OldAngle[INDEX_J1];
		L_TmpAngle[INDEX_J2] = L_OldAngle[INDEX_J2];
        
        //Deside Left arm action
        switch (L_CmdState.Action)
        {
        case CMD_STICK_MOVE:
            L_MParam.DstVel[INDEX_POS_X] = HAND_MAXSPEED * L_CmdState.Stick[INDEX_POS_X];
            L_MParam.DstVel[INDEX_POS_Y] = HAND_MAXSPEED * L_CmdState.Stick[INDEX_POS_Y];

            Ret = CalcHandSpeedToAngularVel(&L_MParam);
            if (fabs(L_MParam.DstAngle[INDEX_J2]) > SINGULARITY_THRESH * 5.0) {
                pub_msg.data[INDEX_J1] = L_MParam.DstAngle[INDEX_J1];
                pub_msg.data[INDEX_J2] = L_MParam.DstAngle[INDEX_J2];
                L_OldAngle[INDEX_J1] = L_MParam.DstAngle[INDEX_J1];
                L_OldAngle[INDEX_J2] = L_MParam.DstAngle[INDEX_J2];
            }else{
                pub_msg.data[INDEX_J1] = L_TmpAngle[INDEX_J1];
                pub_msg.data[INDEX_J2] = L_TmpAngle[INDEX_J2];
            }
            
            if(L_CmdState.Touch == TOUCH_ON){
                //printf("touch!\n");
                pub_msg.data[INDEX_J3] = GRIPP_ON_ANGLE;
            }else{
                pub_msg.data[INDEX_J3] = GRIPP_OFF_ANGLE;
            }
            
            break;
        
        case CMD_FLICK_UP:
            if (L_PParam.PlanningState == STATE_STANDBY) {
				L_MParam.DstPos[INDEX_POS_X] = L_MParam.Pos[INDEX_POS_X] + FLICK_LENGTH;
				L_MParam.DstPos[INDEX_POS_Y] = L_MParam.Pos[INDEX_POS_Y];

                L_MParam.VelRatio = 2.0;
                L_PParam.PlanningState = STATE_PLANNING;
			}
        case CMD_FLICK_DOWN:
            if (L_PParam.PlanningState == STATE_STANDBY) {
				L_MParam.DstPos[INDEX_POS_X] = L_MParam.Pos[INDEX_POS_X] - FLICK_LENGTH;
				L_MParam.DstPos[INDEX_POS_Y] = L_MParam.Pos[INDEX_POS_Y];

                L_MParam.VelRatio = 2.0;
                L_PParam.PlanningState = STATE_PLANNING;
			}
        case CMD_FLICK_LEFT:
            if (L_PParam.PlanningState == STATE_STANDBY) {
				L_MParam.DstPos[INDEX_POS_X] = L_MParam.Pos[INDEX_POS_X];
				L_MParam.DstPos[INDEX_POS_Y] = L_MParam.Pos[INDEX_POS_Y] + FLICK_LENGTH;

                L_MParam.VelRatio = 2.0;
                L_PParam.PlanningState = STATE_PLANNING;
			}
        case CMD_FLICK_RIGHT:
            if (L_PParam.PlanningState == STATE_STANDBY) {
				L_MParam.DstPos[INDEX_POS_X] = L_MParam.Pos[INDEX_POS_X];
				L_MParam.DstPos[INDEX_POS_Y] = L_MParam.Pos[INDEX_POS_Y] - FLICK_LENGTH;

                L_MParam.VelRatio = 2.0;
                L_PParam.PlanningState = STATE_PLANNING;
			}
            
			if (L_PParam.PlanningState == STATE_PLANNING) {
				//printf("start planning\n");
                ExecMovePosStraight(&L_PParam, &L_MParam, L_MParam.VelRatio);
			}
			else if (L_PParam.PlanningState == STATE_EXECUTE) {
				ExecMovePosStraight(&L_PParam, &L_MParam, L_MParam.VelRatio);
				pub_msg.data[INDEX_J1] = L_MParam.DstAngle[INDEX_J1];
                pub_msg.data[INDEX_J2] = L_MParam.DstAngle[INDEX_J2];
                pub_msg.data[INDEX_J3] = GRIPP_ON_ANGLE;
			}
			else if (L_PParam.PlanningState == STATE_FINISH) {
				//printf("finish planning\n");
                L_PParam.PlanningState = STATE_STANDBY;
				pub_msg.data[INDEX_J3] = GRIPP_OFF_ANGLE;
                L_CmdState.Action = CMD_NONE;
			}
			else {}

            break;
        case CMD_ZERO_POTISION:
            pub_msg.data[INDEX_J1] = 0.0;
            pub_msg.data[INDEX_J2] = 0.0;
            L_OldAngle[INDEX_J1] = 0.0;
            L_OldAngle[INDEX_J2] = 0.0;
            break;
        default:
            pub_msg.data[INDEX_J1] = L_TmpAngle[INDEX_J1];
            pub_msg.data[INDEX_J2] = L_TmpAngle[INDEX_J2];
            
            if(L_CmdState.Touch == TOUCH_ON){
                //printf("touch!\n");
                pub_msg.data[INDEX_J3] = GRIPP_ON_ANGLE;
            }else{
                pub_msg.data[INDEX_J3] = GRIPP_OFF_ANGLE;
            }
            break;
        }

        //Right Arm
        R_TmpAngle[INDEX_J1] = R_OldAngle[INDEX_J1];
		R_TmpAngle[INDEX_J2] = R_OldAngle[INDEX_J2];
        
        //Deside Right arm action
        switch (R_CmdState.Action)
        {
        case CMD_STICK_MOVE:
            R_MParam.DstVel[INDEX_POS_X] = HAND_MAXSPEED * R_CmdState.Stick[INDEX_POS_X];
            R_MParam.DstVel[INDEX_POS_Y] = HAND_MAXSPEED * R_CmdState.Stick[INDEX_POS_Y];

            Ret = CalcHandSpeedToAngularVel(&R_MParam);
            if (fabs(R_MParam.DstAngle[INDEX_J2]) > SINGULARITY_THRESH * 5.0) {
                pub_msg.data[INDEX_J1 + 3] = R_MParam.DstAngle[INDEX_J1];
                pub_msg.data[INDEX_J2 + 3] = R_MParam.DstAngle[INDEX_J2];
                R_OldAngle[INDEX_J1] = R_MParam.DstAngle[INDEX_J1];
                R_OldAngle[INDEX_J2] = R_MParam.DstAngle[INDEX_J2];
            }else{
                pub_msg.data[INDEX_J1 + 3] = R_TmpAngle[INDEX_J1];
                pub_msg.data[INDEX_J2 + 3] = R_TmpAngle[INDEX_J2];
            }
            
            if(R_CmdState.Touch == TOUCH_ON){
                //printf("touch!\n");
                pub_msg.data[INDEX_J3 + 3] = GRIPP_ON_ANGLE;
            }else{
                pub_msg.data[INDEX_J3 + 3] = GRIPP_OFF_ANGLE;
            }
            
            break;
        
        case CMD_FLICK_UP:
            if (R_PParam.PlanningState == STATE_STANDBY) {
				R_MParam.DstPos[INDEX_POS_X] = R_MParam.Pos[INDEX_POS_X] + FLICK_LENGTH;
				R_MParam.DstPos[INDEX_POS_Y] = R_MParam.Pos[INDEX_POS_Y];

                R_MParam.VelRatio = 2.0;
                R_PParam.PlanningState = STATE_PLANNING;
			}
        case CMD_FLICK_DOWN:
            if (R_PParam.PlanningState == STATE_STANDBY) {
				R_MParam.DstPos[INDEX_POS_X] = R_MParam.Pos[INDEX_POS_X] - FLICK_LENGTH;
				R_MParam.DstPos[INDEX_POS_Y] = R_MParam.Pos[INDEX_POS_Y];

                R_MParam.VelRatio = 2.0;
                R_PParam.PlanningState = STATE_PLANNING;
			}
        case CMD_FLICK_LEFT:
            if (R_PParam.PlanningState == STATE_STANDBY) {
				R_MParam.DstPos[INDEX_POS_X] = R_MParam.Pos[INDEX_POS_X];
				R_MParam.DstPos[INDEX_POS_Y] = R_MParam.Pos[INDEX_POS_Y] + FLICK_LENGTH;

                R_MParam.VelRatio = 2.0;
                R_PParam.PlanningState = STATE_PLANNING;
			}
        case CMD_FLICK_RIGHT:
            if (R_PParam.PlanningState == STATE_STANDBY) {
				R_MParam.DstPos[INDEX_POS_X] = R_MParam.Pos[INDEX_POS_X];
				R_MParam.DstPos[INDEX_POS_Y] = R_MParam.Pos[INDEX_POS_Y] - FLICK_LENGTH;

                R_MParam.VelRatio = 2.0;
                R_PParam.PlanningState = STATE_PLANNING;
			}
            
			if (R_PParam.PlanningState == STATE_PLANNING) {
				//printf("start planning\n");
                ExecMovePosStraight(&R_PParam, &R_MParam, R_MParam.VelRatio);
			}
			else if (R_PParam.PlanningState == STATE_EXECUTE) {
				ExecMovePosStraight(&R_PParam, &R_MParam, R_MParam.VelRatio);
				pub_msg.data[INDEX_J1 + 3] = R_MParam.DstAngle[INDEX_J1];
                pub_msg.data[INDEX_J2 + 3] = R_MParam.DstAngle[INDEX_J2];
                pub_msg.data[INDEX_J3 + 3] = GRIPP_ON_ANGLE;
			}
			else if (R_PParam.PlanningState == STATE_FINISH) {
				//printf("finish planning\n");
                R_PParam.PlanningState = STATE_STANDBY;
				pub_msg.data[INDEX_J3 + 3] = GRIPP_OFF_ANGLE;
                R_CmdState.Action = CMD_NONE;
			}
			else {}

            break;
        case CMD_ZERO_POTISION:
            pub_msg.data[INDEX_J1 + 3] = 0.0;
            pub_msg.data[INDEX_J2 + 3] = 0.0;
            R_OldAngle[INDEX_J1] = 0.0;
            R_OldAngle[INDEX_J2] = 0.0;
            break;
        default:
            pub_msg.data[INDEX_J1 + 3] = R_TmpAngle[INDEX_J1];
            pub_msg.data[INDEX_J2 + 3] = R_TmpAngle[INDEX_J2];
            
            if(R_CmdState.Touch == TOUCH_ON){
                //printf("touch!\n");
                pub_msg.data[INDEX_J3 + 3] = GRIPP_ON_ANGLE;
            }else{
                pub_msg.data[INDEX_J3 + 3] = GRIPP_OFF_ANGLE;
            }
            break;
        }

        pub.publish(pub_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return Ret;
}

static void joy_callback(const sensor_msgs::Joy &joy_msg)
{
    int i = 0;
    int L_AxisFlag = 0;
    int R_AxisFlag = 0;

    //Left arm
    if(L_PParam.PlanningState == STATE_STANDBY){
        L_CmdState.Action = CMD_NONE;
    }

    // check stick input
    if(fabs(joy_msg.axes[0]) > STICK_THRESH || 
        fabs(joy_msg.axes[1]) > STICK_THRESH){

            L_CmdState.Action = CMD_STICK_MOVE;
            L_CmdState.Stick[0] = joy_msg.axes[1];
            L_CmdState.Stick[1] = joy_msg.axes[0];    
    }else{
        L_CmdState.Stick[0] = 0.0;
        L_CmdState.Stick[1] = 0.0;   
    }

    L_AxisFlag = fabs(joy_msg.axes[4]) + fabs(joy_msg.axes[5]);
    if(L_AxisFlag > 0){
        
        if((int)(joy_msg.axes[4]) == 1){
            L_CmdState.Action = CMD_FLICK_LEFT;
        }else if((int)(joy_msg.axes[4]) == -1){
            L_CmdState.Action = CMD_FLICK_RIGHT;
        }else{} 
        
        if((int)(joy_msg.axes[5]) == 1){
            L_CmdState.Action = CMD_FLICK_UP;
        }else if((int)(joy_msg.axes[5]) == -1){
            L_CmdState.Action = CMD_FLICK_DOWN;
        }else{}

    }else{}
        
    if(joy_msg.buttons[4] == TOUCH_ON){
        //printf("touch!\n");
        L_CmdState.Touch = TOUCH_ON;
    }else{
        L_CmdState.Touch = TOUCH_OFF;
    }
    
    if(joy_msg.buttons[8] == 1){
        L_CmdState.Action = CMD_ZERO_POTISION;
    }else{}

    //Right arm
    if(R_PParam.PlanningState == STATE_STANDBY){
        R_CmdState.Action = CMD_NONE;
    }

    // check stick input
    if(fabs(joy_msg.axes[2]) > STICK_THRESH || 
        fabs(joy_msg.axes[3]) > STICK_THRESH){

            R_CmdState.Action = CMD_STICK_MOVE;
            R_CmdState.Stick[0] = joy_msg.axes[2];
            R_CmdState.Stick[1] = joy_msg.axes[3];    
    }else{
        R_CmdState.Stick[0] = 0.0;
        R_CmdState.Stick[1] = 0.0;   
    }

    for(i =0; i < 4; i++){
        R_AxisFlag += joy_msg.buttons[i];
    }
    
    if(R_AxisFlag > 0){
        
        if(joy_msg.buttons[0] == 1){
            R_CmdState.Action = CMD_FLICK_LEFT;
        }else if(joy_msg.buttons[1] == 1){
            R_CmdState.Action = CMD_FLICK_UP;
        }else if(joy_msg.buttons[2] == 1){
            R_CmdState.Action = CMD_FLICK_DOWN;
        }else if(joy_msg.buttons[3] == 1){
            R_CmdState.Action = CMD_FLICK_RIGHT;
        }else{}

    }else{}
        
    if(joy_msg.buttons[5] == TOUCH_ON){
        //printf("touch!\n");
        R_CmdState.Touch = TOUCH_ON;
    }else{
        R_CmdState.Touch = TOUCH_OFF;
    }

    if(joy_msg.buttons[9] == 1){
        R_CmdState.Action = CMD_ZERO_POTISION;
    }else{}
}

static void chatterCallback(const robot_state_msg::RobotState& msg){
    //printf("subscribe! J1: %f\n", msg.cur_angle[INDEX_J1]);
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

}

static void chatterCallback2(const robot_state_msg::RobotState& msg){
    //printf("subscribe! J1: %f\n", msg.cur_angle[INDEX_J1]);
    //Right hand state
    //angle
    R_MParam.CurAngle[INDEX_J1] = msg.cur_angle[INDEX_J1];
    R_MParam.CurAngle[INDEX_J2] = msg.cur_angle[INDEX_J2];
    
    //end effector pose
    R_MParam.Pos[INDEX_POS_X] = msg.pos_x;
    R_MParam.Pos[INDEX_POS_Y] = msg.pos_y;
    R_MParam.Pos[INDEX_POS_Z] = msg.pos_z;
    
    //arm mode
    R_MParam.ArmMode = msg.arm_mode;
    
    //gripp state
    R_MParam.GrippState = msg.gripp_state;
    
}