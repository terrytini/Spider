#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "ax12a.h"

#define DEFAULT_BAUDRATE 1000000
#define USB_LATENCY 4000

static int fd;

int openPort(char *portName)
{
    //int terminal_fd;
    struct termios terminal_io;

    fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(fd < 0)
    {
        printf("ax12a: Error opening serial port \"%s\"\n", portName);
        return 0;
    }

    memset(&terminal_io, 0, sizeof(terminal_io));

    terminal_io.c_cflag = B1000000 | CS8 | CLOCAL | CREAD;
    // input modes (IGNPAR: Ignore framing errors and parity errors)
    terminal_io.c_iflag = IGNPAR;
    // output modes
    terminal_io.c_oflag = 0;
    // local modes
    terminal_io.c_lflag = 0;
    // special characters
    terminal_io.c_cc[VTIME] = 0;
    terminal_io.c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &terminal_io);

    return fd;
}

int closePort()
{
    return close(fd);
}

float toDegreeFloat(int degreeValue)
{
    return (degreeValue * 300.0) / 0x3ff;
}

float toRPMFloat(int speedValue)
{
    return (speedValue * 114.0) / 0x3ff;
}

int sendReceive(int fd, unsigned char *send_packet, int length, unsigned char *return_packet)
{
    int i;
    unsigned char buffer[20];

    // Send a packet

    int retval = write(fd, send_packet, length);

    // Wait for a return packet

    usleep((10 * 9) + (USB_LATENCY * 2) + (2000));

    // Loop until a packet is received (for now)
    // Need to implement timeout

    retval = 0;

    for(i = 0; i < 10000; i++)
    {
	retval = read(fd, return_packet, 20);
	if(retval != 0)
	{
	    break;
	}
    }

    return retval;
}

void printResponse(unsigned char *packet, int length)
{
    int i;

    printf("Response: ");

    for(i = 0; i < length; i++)
    {
	printf("0x%02X ", packet[i] & 0xFF);
    }

    printf("\n");
}

unsigned char getChecksum(unsigned char *packet, int startIndex, int endIndex)
{
    int checksum = 0, i;

    for(i = startIndex; i < endIndex; i++)
    {
	checksum += packet[i];
    }

    return (~checksum) & 0xff;
}

int readTwoByteLH(int fd, int id, unsigned char addr)
{
    unsigned char buffer[20];
    unsigned char packet[8];

    packet[0] = 0xff;
    packet[1] = 0xff;
    packet[2] = id;	// id of ax-12+
    packet[3] = 0x04;	// Length
    packet[4] = 0x02;	// Instruction (READ DATA)
    packet[5] = addr;	// Parameter 1: Starting address
    packet[6] = 0x02;	// Parameter 2: Length of data to be read
    packet[7] = getChecksum(packet, 2, 7);
    
    int length = sendReceive(fd, packet, 8, buffer);

    if(length == 0)
    {
	return -1;
    }
    else if(buffer[4] != 0)
    {
	return 0xffffff80 | buffer[4];
    }
    else
    {
	return (buffer[6] << 8) | buffer[5];
    }
}

int getPresentPositionSpeed(int id, float *position, float *speed)
{
    unsigned char buffer[20];
    unsigned char packet[8];

    packet[0] = 0xff;
    packet[1] = 0xff;
    packet[2] = id;	// id of ax-12+
    packet[3] = 0x04;	// Length
    packet[4] = 0x02;	// Instruction (READ DATA)
    packet[5] = 0x24;	// Parameter 1: Starting address
    packet[6] = 0x04;	// Parameter 2: Length of data to be read
    packet[7] = getChecksum(packet, 2, 7);

    int length = sendReceive(fd, packet, 8, buffer);

    if(length == 0)
    {
	return -1;
    }
    else if(buffer[4] != 0)
    {
	return 0xffffff80 | buffer[4];
    }
    else
    {
	*position = toDegreeFloat((buffer[6] << 8) | buffer[5]);
	*speed = toRPMFloat((buffer[8] << 8) | buffer[7]);
	return 0;
    }
}

int readOneByte(int fd, int id, unsigned char addr)
{
    unsigned char buffer[20];
    unsigned char packet[8];

    packet[0] = 0xff;
    packet[1] = 0xff;
    packet[2] = id;	// id of ax-12+
    packet[3] = 0x04;	// Length
    packet[4] = 0x02;	// Instruction (READ DATA)
    packet[5] = addr;	// Parameter 1: Starting address
    packet[6] = 0x02;	// Parameter 2: Length of data to be read
    packet[7] = getChecksum(packet, 2, 7);
    
    int length = sendReceive(fd, packet, 8, buffer);

    if(length == 0)
    {
	return -1;
    }
    else if(buffer[4] != 0)
    {
	return 0xffffff80 | buffer[4];
    }
    else
    {
	return buffer[5];
    }
}

int ping(int fd, int id)
{
    unsigned char buffer[20];
    unsigned char packet[6];

    packet[0] = 0xff;
    packet[1] = 0xff;
    packet[2] = id;	// id of ax-12+
    packet[3] = 0x02;	// Length
    packet[4] = 0x01;	// Instruction (PING)
    packet[5] = getChecksum(packet, 2, 5);

    int length = sendReceive(fd, packet, 6, buffer);

    if(length == 0)
    {
	return -1;
    }
    else if(buffer[4] != 0)
    {
	return 0xffffff80 | buffer[4];
    }
    else
    {
	return buffer[4];
    }
}

int getModelNumber(int fd, int id)
{
    return readTwoByteLH(fd, id, 0x00);
}

int getPresentPosition(int id)
{
    return readTwoByteLH(fd, id, 0x24);
}

int getPresentSpeed(int id)
{
    return readTwoByteLH(fd, id, 0x26);
}

int getPresentLoad(int id)
{
    return readTwoByteLH(fd, id, 0x28);
}

int getPresentVoltage(int id)
{
    return readOneByte(fd, id, 0x2a);
}

int isMoving(int id)
{
    return readOneByte(fd, id, 0x2e);
}

int turnMotor(int id, float degree, float speed)
{
    int degValue = (int) ((0x3ff / 300.0) * degree);
    int speedValue = (int) ((0x3ff / 114.0) * speed);
    unsigned char packet[11];
    unsigned char buffer[20];

    packet[0] = 0xff;			// Start packet (always 0xff)
    packet[1] = 0xff;			// Start packet (always 0xff)
    packet[2] = id;			// Motor ID 0 - 254
    packet[3] = 0x07;			// Packet length (num parameters + 2)
    packet[4] = 0x03;			// Instruction (3 for WRITE DATA)
    packet[5] = 0x1e;			// Param 1: Address (0x1e for Goal Position)
    packet[6] = degValue & 0xff;	// Param 2: Low byte of the goal position
    packet[7] = (degValue >> 8) & 0xff;	// Param 3: High byte of the goal position
    packet[8] = speedValue & 0xff;
    packet[9] = (speedValue >> 8) & 0xff;

    // Calculate checksum = ~(ID + Length + Instruction + Param 1 + ... + Param N)

    packet[10] = getChecksum(packet, 2, 10);

    return sendReceive(fd, packet, 11, buffer);
}

int sendRegWrite(int fd, int motorID, float degree, float speed)
{
    int degValue = (int) ((0x3ff / 300.0) * degree);
    int speedValue = (int) ((0x3ff / 114.0) * speed);
    unsigned char packet[11];
    unsigned char buffer[20];

    packet[0] = 0xff;				// Start packet (always 0xff)
    packet[1] = 0xff;				// Start packet (always 0xff)
    packet[2] = motorID;			// Motor ID 0 - 253
    packet[3] = 0x07;				// Packet length (num parameters + 2)
    packet[4] = 0x04;				// Instruction (4 for REG_WRITE)
    packet[5] = 0x1e;				// Param 1: Address (0x1e for Goal Position)
    packet[6] = degValue & 0xff;		// Param 2: Low byte of the goal position
    packet[7] = (degValue >> 8) & 0xff;		// Param 3: High byte of the goal position
    packet[8] = speedValue & 0xff;		// Param 4: Low byte of the goal speed
    packet[9] = (speedValue >> 8) & 0xff;	// Param 5: High byte of the goal speed

    // Calculate checksum = ~(ID + Length + Instruction + Param 1 + ... + Param N)

    packet[10] = getChecksum(packet, 2, 10);

    return sendReceive(fd, packet, 11, buffer);
}

int sendAction(int fd)
{
    unsigned char packet[6];
    unsigned char buffer[20];

    packet[0] = 0xff;			// Start packet (always 0xff)
    packet[1] = 0xff;			// Start packet (always 0xff)
    packet[2] = 0xfe;			// Motor ID 0xfe (broadcast ID)
    packet[3] = 0x02;			// Packet length (num parameters + 3)
    packet[4] = 0x5;			// Instruction (5 for ACTION)

    // Calculate checksum = ~(ID + Length + Instruction + Param 1 + ... + Param N)

    packet[5] = getChecksum(packet, 2, 5);

    return sendReceive(fd, packet, 6, buffer);
}

float movingTimeMS(float degree, float rpm)
{
    return (degree * 60000.0) / (rpm * 360);
}

void waitUntilStop(int id)
{
    while(isMoving(id));
}

void waitSync(int *id, int numMotors)
{
    int i;

    for(i = 0; i < numMotors; i++)
    {
	    while(isMoving(id[i]));
    }
}
