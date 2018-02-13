#include "PID.h"
#define encoder1PinA  3
#define encoder1PinB  2
#define encoder2PinA  21  
#define encoder2PinB  20  

#define In1 8
#define In2 9
#define In3 10
#define In4 11

#define EnA 5
#define EnB 6

#define LOOPTIME 300
#define MaxPWM 255

static double MaxSpeed = 15;

int pinAState1 = 0;
int pinAState1Old = 0;
int pinBState1 = 0;
int pinBState1Old = 0;

int pinAState2 = 0;
int pinAState2Old = 0;
int pinBState2 = 0;
int pinBState2Old = 0;

char commandArray[4];


volatile long EncoderposR = 0;
volatile long EncoderposL = 0;
//volatile long unknownvalue = 0;

volatile int lastEncoded = 0;
unsigned long lastMilli = 0;     // loop timing
unsigned long lastRecvMilli = 0;
long dT = 0;
//unsigned long cc = 0;

double omega_target_R = 0.0;
double omega_target_L = 0.0;
double omega_actual_R = 0.0;
double omega_actual_L = 0.0;
int PWM_val_R = 0;
int PWM_val_L = 0;

int PWM_val = 0; 
 // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
 //GM25-370直流碳刷电机
int CPR = 4;    // encoder count per revolution
int gear_ratio = 75;
int actual_send = 0;
int target_receive_R = 0;
int target_receive_L = 0;

//int analogPin = A0;
//unsigned int current = 0;

PID pid;

//SerialDataFloat yaw;

unsigned long lastBool = 0;
String feedback;
unsigned long lastFeedBack = 0;

char rec;
bool recieveComplete = false;
String recbuf;

bool checksum(float firstpara, float secondpara, float sum);
//bool left_motor_protection(unsigned int current);
//bool right_motor_protection(unsigned int current);
//void disableCurrentProtect(bool cmd);

typedef union{
	double Data;
	byte binary[4];
}SerialDataDouble;

typedef union{
	float Data;
	byte binary[4];
}SerialDataFloat;

typedef struct{
	SerialDataDouble Kp;
	SerialDataDouble Ki;
	SerialDataDouble Kd;
}SerialDataParamPack;
