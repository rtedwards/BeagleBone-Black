/*
 *  Author:  Robert Edwards
 *  For use with Epson G350 IMU
 */
 
 
#ifndef EpsonIMU_H_
#define EpsonIMU_H_

#include <stdint.h>

//#define EPSON_IMU "/dev/ttyO1"
#define EPSON_DATA_RDY_PIN 7	//GPIO_7 pin 42


//static uint8_t new_data_flag;
static uint8_t imu_status_bit_5;
static uint8_t imu_status_bit_6;

static int16_t E_temperature_data;
static int16_t E_gyro_x_data;
static int16_t E_gyro_y_data;
static int16_t E_gyro_z_data;
static int16_t E_accel_x_data;
static int16_t E_accel_y_data;
static int16_t E_accel_z_data;
static uint16_t E_gpio_data;
static uint16_t E_glob_cmd;
static uint16_t E_imu_status;

static float E_temperature;  //0.0042725*(sensor_temp + 15214) + 25
static float E_gyro_x;  //G [deg/s] = 0.0125 * gyro_data
static float E_gyro_y;  //G [deg/s] = 0.0125 * gyro_data
static float E_gyro_z;  //G [deg/s] = 0.0125 * gyro_data
static float E_accel_x;  //G [mG] = 0.125 * accel_data
static float E_accel_y;  //G [mG] = 0.125 * accel_data
static float E_accel_z;  //G [mG] = 0.125 * accel_data


int checkStartUp();
int checkStatus();
void setSamplesPerSec(uint8_t rate);
void setTAP(uint8_t tap);
void samplingMode();
void manualMode();
void configurationMode();
int readBurstData();
int setEpsonIMU(const char *Device,const unsigned int Bauds);
int readEpsonIMU();
void printEpsonIMU();

float getEpsonTemperature();
float getEpsonGyroX();
float getEpsonGyroY();
float getEpsonGyroZ();
float getEpsonAccelX();
float getEpsonAccelY();
float getEpsonAccelZ();

#endif








