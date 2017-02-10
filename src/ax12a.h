#ifndef _AX12A_H_
#define _AX12A_H_

int openPort(char *portName);
int closePort();
int getModelNumber(int id);
int getPresentPosition(int id);
int getPresentSpeed(int id);
int getPresentLoad(int id);
int getPresentVoltage(int id);
int getPresentPositionSpeed(int id, float *position, float *speed);
int isMoving(int id);

int ping(int id);

float toDegreeFloat(int degreeValue);
float toRPMFloat(int speedValue);

void waitUntilStop(int id);
void waitSync(int *id, int numMotors);

int turnMotor(int id, float degree, float speed);
int sendRegWrite(int id, float degree, float speed);
int sendAction();

#endif
