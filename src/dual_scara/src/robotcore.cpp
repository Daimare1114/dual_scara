#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "robot_state_msg/RobotState.h"
#include "../include/dual_scara/robot_kinematics.h"
#include "../include/dual_scara/PCA9685.h"

/*Prototype function*/
void chatterCallback(const std_msgs::Float32MultiArray& msg);
int CalcCmdJ1Angle(float DstJ1Angle, int OffsetJ1Angle);
int CalcCmdJ2Angle(float DstJ1Angle, int OffsetJ1Angle);
int CalcCmdJ2AngleReverse(float DstJ1Angle, int OffsetJ1Angle);
int CalcCmdJ3Angle(float DstJ1Angle, int OffsetJ1Angle);
int CalcCmdJ3AngleReverse(float DstJ1Angle, int OffsetJ1Angle);
/********************/

MotorParam L_Param;
MotorParam R_Param;
PCA9685 Motor;

int main(int argc, char** argv)
{
  int i = 0;
  float Tmp = 0.0;
  FILE *fp;

  ros::init(argc, argv, "robotcore");
  ros::NodeHandle nh;
  //"topic name, buffer size, callback function"
  ros::Subscriber sub = nh.subscribe("robot_angle", 2, chatterCallback);

  ros::Publisher pub = nh.advertise<robot_state_msg::RobotState>("robot_state", 2);
  robot_state_msg::RobotState pub_msg;
  ros::Publisher pub2 = nh.advertise<robot_state_msg::RobotState>("robot_state2", 2);
  robot_state_msg::RobotState pub_msg2;

  /*Initialize MotorDiver*/
  Motor.init(1, 0x40);
	ROS_INFO("Initialized Moror driver");
  Motor.setPWMFreq(60);

  /*Initialize Angle value*/
  for(i = 0; i < ANGLE_NUM; i++){
    L_Param.OffsetAngle[i] = 0;
    L_Param.InitAngle[i] = 0.0;
    L_Param.CurAngle[i] = 0.0;
    L_Param.CmdAngle[i] = SERVO_MIN_PLS;

    R_Param.OffsetAngle[i] = 0;
    R_Param.InitAngle[i] = 0.0;
    R_Param.CurAngle[i] = 0.0;
    R_Param.CmdAngle[i] = SERVO_MIN_PLS;
  }
  L_Param.InitAngle[0] = INIT_J1;
  L_Param.InitAngle[1] = INIT_J2;
  L_Param.InitAngle[2] = INIT_J3;
  L_Param.CurAngle[0] = INIT_J1;
  L_Param.CurAngle[1] = INIT_J2;
  L_Param.CurAngle[2] = INIT_J3;

  R_Param.InitAngle[0] = -INIT_J1;
  R_Param.InitAngle[1] = -INIT_J2;
  R_Param.InitAngle[2] = INIT_J3;
  R_Param.CurAngle[0] = -INIT_J1;
  R_Param.CurAngle[1] = -INIT_J2;
  R_Param.CurAngle[2] = INIT_J3;

  /*Initialize Pose*/
  CalcForwardKinematics(&L_Param);
  L_Param.Pos[INDEX_POS_Z] = 100.0;//zは任意
  CalcForwardKinematics(&R_Param);
  R_Param.Pos[INDEX_POS_Z] = 100.0;//zは任意

  /*Initialize Gripp State*/
  L_Param.GrippState = GRIPP_OFF;
  R_Param.GrippState = GRIPP_OFF;

  //Calc Command Angle
  L_Param.CmdAngle[0] = CalcCmdJ1Angle(L_Param.InitAngle[0], L_Param.OffsetAngle[0]);
  L_Param.CmdAngle[1] = CalcCmdJ2Angle(L_Param.InitAngle[1], L_Param.OffsetAngle[1]);
  L_Param.CmdAngle[2] = CalcCmdJ3Angle(L_Param.InitAngle[2], L_Param.OffsetAngle[2]);

  R_Param.CmdAngle[0] = CalcCmdJ1Angle(R_Param.InitAngle[0], R_Param.OffsetAngle[0]);
  R_Param.CmdAngle[1] = CalcCmdJ2AngleReverse(R_Param.InitAngle[1], R_Param.OffsetAngle[1]);
  R_Param.CmdAngle[2] = CalcCmdJ3AngleReverse(R_Param.InitAngle[2], R_Param.OffsetAngle[2]);

  //Send Command Angle
  Motor.setPWM(0, 0, (int)L_Param.CmdAngle[0]);
	Motor.setPWM(1, 0, (int)L_Param.CmdAngle[1]);
  Motor.setPWM(2, 0, (int)L_Param.CmdAngle[2]);
  Motor.setPWM(3, 0, (int)R_Param.CmdAngle[0]);
	Motor.setPWM(4, 0, (int)R_Param.CmdAngle[1]);
  Motor.setPWM(5, 0, (int)R_Param.CmdAngle[2]);

  /***********/
  /*Main Loop*/
  /***********/
  ros::Rate loop_rate(MOTOR_HZ);//[Hz]
  while(ros::ok()){
    
    //Send CmdAngle to ServoMotor
    //ROS_INFO("%i, %i, %i", L_Param.CmdAngle[0], L_Param.CmdAngle[1], L_Param.CmdAngle[2]);
    Motor.setPWM(0, 0, (int)L_Param.CmdAngle[0]);
		Motor.setPWM(1, 0, (int)L_Param.CmdAngle[1]);
    Motor.setPWM(2, 0, (int)L_Param.CmdAngle[2]);
    Motor.setPWM(3, 0, (int)R_Param.CmdAngle[0]);
		Motor.setPWM(4, 0, (int)R_Param.CmdAngle[1]);
    Motor.setPWM(5, 0, (int)R_Param.CmdAngle[2]);

    //update arm pose
    CalcForwardKinematics(&L_Param);
    CalcForwardKinematics(&R_Param);
    
    //update robot state
    //left arm
    //angle
    pub_msg.cur_angle[INDEX_J1] = L_Param.CurAngle[INDEX_J1];
    pub_msg.cur_angle[INDEX_J2] = L_Param.CurAngle[INDEX_J2];
    pub_msg.cur_angle[INDEX_J3] = L_Param.CurAngle[INDEX_J3];
    //end effector pose
    pub_msg.pos_x = L_Param.Pos[INDEX_POS_X];
    pub_msg.pos_y = L_Param.Pos[INDEX_POS_Y];
    pub_msg.pos_z = L_Param.Pos[INDEX_POS_Z];
    if(L_Param.ArmMode == SCARA_POSE_RIGHT){
      pub_msg.arm_mode = SCARA_POSE_RIGHT;
    }else{
      pub_msg.arm_mode = SCARA_POSE_LEFT;
    }
    pub_msg.gripp_state = L_Param.GrippState;
    //right arm
    pub_msg2.cur_angle[INDEX_J1] = R_Param.CurAngle[INDEX_J1];
    pub_msg2.cur_angle[INDEX_J2] = R_Param.CurAngle[INDEX_J2];
    pub_msg2.cur_angle[INDEX_J3] = R_Param.CurAngle[INDEX_J3];
    //end effector pose
    pub_msg2.pos_x = R_Param.Pos[INDEX_POS_X];
    pub_msg2.pos_y = R_Param.Pos[INDEX_POS_Y];
    pub_msg2.pos_z = R_Param.Pos[INDEX_POS_Z];
    if(R_Param.ArmMode == SCARA_POSE_RIGHT){
      pub_msg2.arm_mode = SCARA_POSE_RIGHT;
    }else{
      pub_msg2.arm_mode = SCARA_POSE_LEFT;
    }
    pub_msg2.gripp_state = R_Param.GrippState;

    //publish robot_state
    pub.publish(pub_msg);
    pub2.publish(pub_msg2);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ros::spin();

  return 0;
}

void chatterCallback(const std_msgs::Float32MultiArray& msg)
{
  int i = 0;
  float Tmp = 0.0;
  //printf("receive: %f, %f, %f, %f\n", 
  //            msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
  //left arm
  L_Param.CurAngle[INDEX_J1] = msg.data[INDEX_J1];
  L_Param.CurAngle[INDEX_J2] = msg.data[INDEX_J2];
  L_Param.CurAngle[INDEX_J3] = msg.data[INDEX_J3];
  //right arm
  R_Param.CurAngle[INDEX_J1] = msg.data[INDEX_J1 + 3];
  R_Param.CurAngle[INDEX_J2] = msg.data[INDEX_J2 + 3];
  R_Param.CurAngle[INDEX_J3] = msg.data[INDEX_J3 + 3];

  //Calc CmdAngle
  //left arm
  L_Param.CmdAngle[0] = CalcCmdJ1Angle(L_Param.CurAngle[0], L_Param.OffsetAngle[0]);
  L_Param.CmdAngle[1] = CalcCmdJ2Angle(L_Param.CurAngle[1], L_Param.OffsetAngle[1]);
  L_Param.CmdAngle[2] = CalcCmdJ3Angle(L_Param.CurAngle[2], L_Param.OffsetAngle[2]);
  //right arm
  R_Param.CmdAngle[0] = CalcCmdJ1Angle(R_Param.CurAngle[0], R_Param.OffsetAngle[0]);
  R_Param.CmdAngle[1] = CalcCmdJ2AngleReverse(R_Param.CurAngle[1], R_Param.OffsetAngle[1]);
  R_Param.CmdAngle[2] = CalcCmdJ3AngleReverse(R_Param.CurAngle[2], R_Param.OffsetAngle[2]);
}

int CalcCmdJ1Angle(float DstJ1Angle, int OffsetJ1Angle){
  int CmdAngle = SERVO_MIN_PLS;
  float Tmp = 0.0;
  Tmp = (SERVO_MAX_PLS - SERVO_MIN_PLS) * (DstJ1Angle + MOTOR_OFFSET) / 180 + SERVO_MIN_PLS;
  CmdAngle = int(Tmp) + OffsetJ1Angle;

  if(CmdAngle < SERVO_MIN_PLS){
    CmdAngle = SERVO_MIN_PLS;
  }else if(CmdAngle > SERVO_MAX_PLS){
    CmdAngle = SERVO_MAX_PLS;
  }else{}

  return CmdAngle;
}

int CalcCmdJ2Angle(float DstJ1Angle, int OffsetJ1Angle){
  int CmdAngle = SERVO_MIN_PLS;
  float Tmp = 0.0;
  //Tmp = (SERVO_MIN_PLS - SERVO_MAX_PLS) * (DstJ1Angle + MOTOR_OFFSET) / 180 + SERVO_MAX_PLS;
  Tmp = (SERVO_MIN_PLS - SERVO_MAX_PLS) * (DstJ1Angle) / 180 + SERVO_MIN_PLS;
  CmdAngle = int(Tmp) + OffsetJ1Angle;

  if(CmdAngle < SERVO_MIN_PLS){
    CmdAngle = SERVO_MIN_PLS;
  }else if(CmdAngle > SERVO_MAX_PLS){
    CmdAngle = SERVO_MAX_PLS;
  }else{}

  return CmdAngle;
}

int CalcCmdJ2AngleReverse(float DstJ1Angle, int OffsetJ1Angle){
  int CmdAngle = SERVO_MIN_PLS;
  float Tmp = 0.0;
  //Tmp = (SERVO_MIN_PLS - SERVO_MAX_PLS) * (DstJ1Angle + MOTOR_OFFSET) / 180 + SERVO_MAX_PLS;
  Tmp = (SERVO_MIN_PLS - SERVO_MAX_PLS) * (DstJ1Angle) / 180 + SERVO_MAX_PLS;
  CmdAngle = int(Tmp) + OffsetJ1Angle;

  if(CmdAngle < SERVO_MIN_PLS){
    CmdAngle = SERVO_MIN_PLS;
  }else if(CmdAngle > SERVO_MAX_PLS){
    CmdAngle = SERVO_MAX_PLS;
  }else{}

  return CmdAngle;
}

int CalcCmdJ3Angle(float DstJ1Angle, int OffsetJ1Angle){
  int CmdAngle = SERVO_MIN_PLS;
  float Tmp = 0.0;
  Tmp = (SERVO_MAX_PLS - SERVO_MIN_PLS) * (DstJ1Angle + MOTOR_OFFSET) / 180 + SERVO_MIN_PLS;
  CmdAngle = int(Tmp) + OffsetJ1Angle;

  if(CmdAngle < SERVO_MIN_PLS){
    CmdAngle = SERVO_MIN_PLS;
  }else if(CmdAngle > SERVO_MAX_PLS){
    CmdAngle = SERVO_MAX_PLS;
  }else{}

  return CmdAngle;
}

int CalcCmdJ3AngleReverse(float DstJ1Angle, int OffsetJ1Angle){
  int CmdAngle = SERVO_MIN_PLS;
  float Tmp = 0.0;
  Tmp = (SERVO_MIN_PLS - SERVO_MAX_PLS) * (DstJ1Angle + MOTOR_OFFSET) / 180 + SERVO_MAX_PLS;
  CmdAngle = int(Tmp) + OffsetJ1Angle;

  if(CmdAngle < SERVO_MIN_PLS){
    CmdAngle = SERVO_MIN_PLS;
  }else if(CmdAngle > SERVO_MAX_PLS){
    CmdAngle = SERVO_MAX_PLS;
  }else{}

  return CmdAngle;
}