/*
	Author: Robert Edwards

	This library is made specifically for the Analog Devices IMU.
	This library is made to be used with the BeagleBone Black running 
	Linux Angstrom 3.8.13.  

	SPI details specific to Analog Devices IMU:
	Mode	= 3
	CPOL	= 1
	CPHA	= 1
	Speed	= 10,000,000 Hz
	
	-SPI operations are made using the spidev library.  
	-Right now, what is
	to be transfered to the slave device must be hard coded; it cannot be 
	done on the fly like one would need in closed loop control.  
	-Word size (smallest chunk of bits) is 16 bits because the Analog Devices
	IMU has 16 bit registers.
	-Every word transfered must be followed by 3 transfers of 0x0000.  
	The reason is not clear, but it works with the Analog Devices IMU.
*/

#ifdef __cplusplus
extern "C" {//Allows this C file to be used and compiled with C++ files
#endif

#ifndef AnalogIMU_H_
#define AnalogIMU_H_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))	//Number of data

/***************************************************
					VARIABLES
****************************************************/	

//static const char *device = "/dev/spidev1.0";

float A_temperature;
float A_gyro_x;
float A_gyro_y;
float A_gyro_z;
float A_accel_x;
float A_accel_y;
float A_accel_z;
float A_magnet_x;
float A_magnet_y;
float A_magnet_z;
float A_barometer;

	
	
/***************************************************
					FUNCTIONS
****************************************************/	
int openAnalogSPI(const char *device);	//Opens comms with specified device
void closeAnalogSPI(int fd);	//Closes comms with specified device
void setAnalogSPI(int fd);	//Configures SPI settings specfic to AD IMU
int setAnalogIMU(const char *device);	//Configures AD IMU to proper settings
void readAnalogIMU(int fd);	//Reads and saves Gyro, Accel, Maget, and Temp data
void printAnalogIMU();	//Will print all current data variables

void writeRegister(int fd, uint16_t value);
int16_t readRegister(int fd, uint16_t value);

//***** Get Functions *****//
float getAnalogTemperature();
float getAnalogBarometer();
float getAnalogGyroX();
float getAnalogGyroY();
float getAnalogGyroZ();
float getAnalogAccelX();
float getAnalogAccelY();
float getAnalogAccelZ();
float getAnalogMagnetX();
float getAnalogMagnetY();
float getAnalogMagnetZ();


#endif

#ifdef __cplusplus
}
#endif
