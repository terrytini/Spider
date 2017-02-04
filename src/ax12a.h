#ifndef _AX12A_H_
#define _AX12A_H_

int openPort(char *portName);
int closePort();
int getModelNumber(int fd, int id);
int getPresentPosition(int fd, int id);
int getPresentSpeed(int fd, int id);
int getPresentLoad(int fd, int id);
int getPresentVoltage(int fd, int id);
int getPresentPositionSpeed(int fd, int id, float *position, float *speed);
int isMoving(int fd, int id);

int ping(int fd, int id);

float toDegreeFloat(int degreeValue);
float toRPMFloat(int speedValue);

void waitUntilStop(int fd, int id);
void waitSync(int fd, int *id, int numMotors);

int turnMotor(int id, float degree, float speed);
int sendRegWrite(int fd, int id, float degree, float speed);
int sendAction(int fd);

#endif
