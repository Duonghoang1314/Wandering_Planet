void readSensors();
void moveMotor(int leftSpeed, int rightSpeed, int duration);
void followLine();
void Trace_to_Cross_T_Junction();
void Trace_to_Right_Junction();
void Trace_to_Left_Junction();
void turnLeft();
void turnRight();
int calculateError();
float computePID(float error);
void setLinetrackingBaseSpeed(int setBaseSpeed, int setMinSpeed, int setMaxSpeed);

void followLineB();
void Trace_to_Cross_T_Junction_B();
void Trace_to_Right_Junction_B();
void Trace_to_Left_Junction_B();
void turnLeft_B();
void turnRight_B();