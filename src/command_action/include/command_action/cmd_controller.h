#define NO_ERROR				0
#define LOOP_RATE               200//[Hz]

#define CMD_NONE				0
#define CMD_JOINT_MANUAL		1
#define CMD_POS_PTP_MANUAL		2
#define CMD_POS_STRAIGHT_MANUAL	3
#define CMD_TOUCH				4
#define CMD_FLICK_UP			5
#define CMD_FLICK_DOWN			6
#define CMD_FLICK_RIGHT			7
#define CMD_FLICK_LEFT			8
#define CMD_STICK_MOVE          9
#define CMD_ZERO_POTISION       10

#define TOUCH_ON                1
#define TOUCH_OFF               0

#define STICK_NEUTRAL			(float)(0.0)
#define STICK_MAX				(float)(1.0)
#define STICK_MIN				(float)(-1.0)
#define STICK_THRESH            (float)(0.01)

#define CROSS_KEY_NEUTRAL		0
#define CROSS_KEY_UP			1
#define CROSS_KEY_DOWN			-1
#define CROSS_KEY_LEFT			1
#define CROSS_KEY_RIGHT			-1

#define FLICK_LENGTH            (float)(25.0)

typedef struct{
    int Action;
    int Touch;
    float Stick[2];
    int CrossKey;
    int Button;
} CommandState;