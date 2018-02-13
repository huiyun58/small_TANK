
float omega_L_drv = 0;
float omega_R_drv = 0;


SerialDataFloat yaw;

unsigned long lastBool = 0;
String feedback;
unsigned long lastFeedBack = 0;



char rec;
bool recieveComplete = false;
String recbuf;



bool checksum(float firstpara, float secondpara, float sum);
bool left_motor_protection(unsigned int current);
bool right_motor_protection(unsigned int current);
void disableCurrentProtect(bool cmd);
