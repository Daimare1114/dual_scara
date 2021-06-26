#define ANGLE_NUM				15
#define DIM3					3
#define DIM6					6
#define PI						(float)(3.141592)
#define MOTOR_OFFSET        (float)(90.0)
#define SERVO_MAX_PLS       (float)(622.0)
#define SERVO_MIN_PLS       (float)(163.0)
#define DEG2RAD					(float)(PI / 180.0)
#define RAD2DEG					(float)(180.0 / PI)
#define MOTOR_MAXSPEED			(float)(200)//[deg/s]
#define HAND_MAXSPEED			(float)(200)//[mm/s]
#define MOTOR_HZ				(float)(200)//[Hz]
#define SAMPLING_TIME			(float)(1 / MOTOR_HZ)//[s]

#define LINK1					(float)(75.0)
#define LINK2					(float)(117.5)

#define INIT_J1					(float)(30.0)
#define INIT_J2					(float)(-30.0)
#define INIT_J3					(float)(-40.0)

#define GRIPP_ON				(float)(1.0)
#define GRIPP_OFF				(float)(0.0)
#define GRIPP_ON_ANGLE			(float)(-5.0)
#define GRIPP_OFF_ANGLE			(float)(-40.0)

#define SCARA_POSE_RIGHT		0
#define SCARA_POSE_LEFT			1
#define	SCARA_NOT_SINGULARITY	0
#define	SCARA_SINGULARITY		1
#define SINGULARITY_THRESH		(float)(2.0)

#define IK_MODE_DEST			0
#define IK_MODE_CURRENT			1


#define INDEX_POS_X				0
#define INDEX_POS_Y				1
#define INDEX_POS_Z				2

#define INDEX_J1				0
#define INDEX_J2				1
#define INDEX_J3				2
#define INDEX_J4				3
#define INDEX_J5				4
#define INDEX_J6				5
#define INDEX_J7				6

#define MODE_SCARA				0
#define MODE_6AXIS				1

typedef struct {
    int     CmdAngle[ANGLE_NUM];
    int     OffsetAngle[ANGLE_NUM];
    float   InitAngle[ANGLE_NUM];
    float   CurAngle[ANGLE_NUM];
    float   DstAngle[ANGLE_NUM];
    float	DstPos[DIM3];
    float   Pos[DIM3];
    int     GrippState;
    float	VelRatio;
    int     ArmMode;
    int		SingularityState;
}MotorParam;

int InitRBKinema(MotorParam *Param, int Mode);
void CalcForwardKinematics(MotorParam *Param);
int CalcInverseKinematics(MotorParam *Param, int Mode);