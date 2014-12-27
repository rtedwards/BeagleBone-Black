/*
  Author:  Robert Edwards
  
  UART Configuration Mode: IMU goes into Configuration mode immediately after initialization.
    Use this mode to set configurations and various operational settings.  When done,
    go into Sampling mode.  
  
  UART Sampling Mode:
    -Switch between Manual and Auto by writing bit[0] of UART_CNTRL register. 
    -You can only write MODE_CMD (bit[9:8] of MODE_CNTRL)
                        GPIO_DATA (bit[10:8] of GPIO)
                        SOFT_RST (bit[7] of GLOB_CMD)
    Manual mode (UART_AUTO= 0):
    Auto mode (UART_AUTO= 1):
      -Cannot read from registers in this mode.
    Burst mode: When 0x20 is specified for register address, all data is transmitted
      in succession as response.  
      (Addr, ND, EA, Temp_H, Temp_L, XGyro_H, XGyro_L, YGyro_H, YGyro_L, ZGyro_H, ZGyro_L, 
      XAccel_H, XAccel_L, YAccel_H, YAccel_L, ZAccel_H, ZAccel_L, GPIO_H, GPIO_L, *Count_H, *Count_L, CR)
  
  16-bit registers are little endian [LSB, MSB]
  16-bit data is transfered in big endian [MSB, LSB]

*/


#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include "/home/xuser/BBBB/Libraries/serialib/serialib.h"
#include "/home/xuser/BBBB/Libraries/SimpleGPIO/SimpleGPIO.h"
#include "EpsonIMU.h"

uint8_t FLAG = 0x00;  //ND flag/EA flag (Read)
uint8_t TEMP_OUT = 0x02;  //Temperature sensor output (Read)
uint8_t XGYRO_OUT = 0x04;  //X-axis Gyroscope output (Read)
uint8_t YGYRO_OUT = 0x06;  //Y-axis Gyroscope output (Read)
uint8_t ZGYRO_OUT = 0x08;  //Z-axis Gyroscope output (Read)
uint8_t XACCEL_OUT = 0x0A;  //X-axis Accelerometer output (Read)
uint8_t YACCEL_OUT = 0x0C;  //Y-axis Accelerometer output (Read)
uint8_t ZACCEL_OUT = 0x0E;  //Z-axis Accelerometer output (Read)
uint8_t GPIO_W = 0x10;  //GPIO (Write)
uint8_t GPIO_R = 0x11;  //GPIO read (Read)
uint8_t COUNT = 0x12;  //Sampling counter value (Read)
uint8_t BURST = 0x20;  //UART burst mode (Read)
uint8_t SIG_CTRL_W = 0x32;  //Data Ready signal (Write)
uint8_t SIG_CTRL_R = 0x33;  //Data Ready signal (Read)
uint8_t MSC_CTRL_W = 0x34;  //Other control (Write)
uint8_t MSC_CTRL_R = 0x35;  //Other control (Read)
uint8_t SMPL_CTRL_W = 0x36;  //Sampling Control (Write)
uint8_t SMPL_CTRL_R = 0x37;  //Sampling Control (Read)
uint8_t MODE_CTRL_W = 0x38;  //Operation mode/Filter control (Write)
uint8_t MODE_CTRL_R = 0x39;  //Operation mode/Filter control (Read)
uint8_t UART_CTRL_W = 0x3B;  //UART control (Write)
uint8_t UART_CTRL_R = 0x3A;  //UART control (Read)
uint8_t DIAG_STAT = 0x3C;  //Diagnosis result (Read)
uint8_t GLOB_CMD_W= 0x3E;  //System control (Write)
uint8_t GLOB_CMD_R = 0x3F;  //System control (Read)
uint8_t COUNT_CNTRL_W = 0x50;  //Counter value transmission control (Write)
uint8_t COUNT_CNTRL_R = 0x51;  //Counter value transmission control (Read)

serialib Serial1;	//EPSON_IMU port

int rdy_bit = 1;
int flag;
int Ret;			//Used for return values
char in_byte;

unsigned int led1 = 50;	//GPIO1_18 (1x32 + 18)
unsigned int led2 = 7;	//GPIO0_7 (0x32 + 7)
unsigned int data_rdy_value = 0;	//Associated with an input pin

/* Functions */
int checkStartUp()
{
  	Serial1.writeByte(0x3E);
  	Serial1.writeByte(0x00);
  	Serial1.writeByte(0x0D);
  
  	while(Serial1.Peek() < 4) {
    	printf("Hanging on EpsonIMU: CheckStartup\n");
  	}//end while
  
  	Serial1.readByte(&in_byte, 0);
  	Serial1.readByte(&in_byte, 0);
  	E_glob_cmd = in_byte;	  //MSB
  	Serial1.readByte(&in_byte, 0);
  	E_glob_cmd = (E_glob_cmd << 8) | in_byte;  //LSB
  	Serial1.readByte(&in_byte, 0);
  
  	rdy_bit = (E_glob_cmd >> 10) | 0x00;  //check if bit 10 = 0
  
  	return rdy_bit;
}//end checkStartUp()
  
    
int checkStatus() 
{
  	Serial1.writeByte(0x3C);
  	Serial1.writeByte(0x00);
  	Serial1.writeByte(0x0D);
  
  	while(Serial1.Peek() < 4) {
    	printf(".");
  	}//end while
  
  	Serial1.readByte(&in_byte, 0);
  	Serial1.readByte(&in_byte, 0); 
  	E_imu_status =  in_byte;					//MSB
  	Serial1.readByte(&in_byte, 0);
  	E_imu_status = (E_imu_status << 8) | in_byte;//LSB
  	Serial1.readByte(&in_byte, 0);
  
  	imu_status_bit_5 = (E_imu_status >> 5) | 0x00;  
  	imu_status_bit_6 = (E_imu_status >> 6) | 0x00;
  
  	if (imu_status_bit_5 != 0 || imu_status_bit_6 != 0)
    	return 1;
  
  	return 0;
}//end checkStatus()


void samplingMode()
{
  	Serial1.writeByte(0xB9);  //Sampling mode
  	Serial1.writeByte(0x01);
  	Serial1.writeByte(0x0D);
  	return;
}//end samplingMode()


void setSamplesPerSec(uint8_t rate)
{
	//0x01 - 1000 Sps
	//0x02 - 500 Sps
	//0x03 - 250 Sps
	//0x04 - 125 Sps
	//0x05 - 62.5 Sps
	//0x06 - 31.25 Sps
	//0x07 - 15.625 Sps

	Serial1.writeByte(0xB7);	//Samples Per Second register
	Serial1.writeByte(rate);	//Rate number
	Serial1.writeByte(0x0D);	//End write
	
	printf("Taps: 0x%X\n", rate);
	return;
}//end setSamplesPerSec()

void setTAP(uint8_t tap)
{
	//0x01 - 2 Tap
    //0x02 - 4 Tap
    //0x03 - 8 Tap
    //0x04 - 16 Tap
    //0x05 - 32 Tap
    //0x06 - 64 Tap
    //0x07 - 128 Tap

	Serial1.writeByte(0xB8);	//TAP register
	Serial1.writeByte(tap);		//TAP number
	Serial1.writeByte(0x0D);	//End write

	printf("Taps: 0x%X\n", tap);
	return;
}//end setTAP()

void manualMode()
{
	Serial1.writeByte(0xBA);  //UART manual mode
    Serial1.writeByte(0x00);
    Serial1.writeByte(0x0D);

	return;
}//end manualMode()

void configurationMode()
{
  	Serial1.writeByte(0xB9);
  	Serial1.writeByte(0x02);
  	Serial1.writeByte(0x0D);
  	return;
}//end configurationMode()


int readBurstData()
{
	//Reset Variables
	flag = 0x0000;
	E_temperature_data = 0x0000;
	E_gyro_x_data = 0x0000;
	E_gyro_y_data = 0x0000;
	E_gyro_z_data = 0x0000;
	E_accel_x_data = 0x0000;
	E_accel_y_data = 0x0000;
	E_accel_z_data = 0x0000;
	E_gpio_data = 0x0000;

	//Burst Mode Command
	Serial1.writeByte(0x20);	//Burst register
	Serial1.writeByte(0x00);
	Serial1.writeByte(0x0D);	//End write

	//***** Read Data *****//
	Serial1.readByte(&in_byte, 0);
  	if(in_byte == 0x20)	//Check first byte for 0x20
  	{
		//Flag
  		Ret = Serial1.readByte(&in_byte, 0);
		flag = in_byte;  //MSB
    	Ret = Serial1.readByte(&in_byte, 0);
		flag = (flag << 8) | in_byte;  //LSB
    
		//*** Temperature ***//
		//Read data and parse into one variable
  	  	Ret = Serial1.readByte(&in_byte, 0);
		E_temperature_data = in_byte;  //MSB
		Ret = Serial1.readByte(&in_byte, 0);

		//Convert data to engineering units
		E_temperature_data = (E_temperature_data << 8) | in_byte;  //LSB
		E_temperature = (0.0042725*(E_temperature_data + 15214)) + 25;
		
		//Gyro X
		//Read data and parse into one variable
		Ret = Serial1.readByte(&in_byte, 0);
		E_gyro_x_data = in_byte;  //MSB
		Ret = Serial1.readByte(&in_byte, 0);

		//Convert data to engineering units
		E_gyro_x_data = (E_gyro_x_data << 8) | in_byte;  //LSB
		E_gyro_x = 0.0125*E_gyro_x_data;
		
		//Gyro Y
		//Read data and parse into one variable
		Ret = Serial1.readByte(&in_byte, 0);
		E_gyro_y_data = in_byte;  //MSB
		Ret = Serial1.readByte(&in_byte, 0);

		//Convert data to engineering units
		E_gyro_y_data = (E_gyro_y_data << 8) | in_byte;  //LSB
		E_gyro_y = 0.0125*E_gyro_y_data;
		
		//Gyro Z
		//Read data and parse into one variable
		Ret = Serial1.readByte(&in_byte, 0);
		E_gyro_z_data = in_byte;  //MSB
		Ret = Serial1.readByte(&in_byte, 0);
		
		//Convert data to engineering units
		E_gyro_z_data = (E_gyro_z_data << 8) | in_byte;  //LSB
		E_gyro_z = 0.0125*E_gyro_z_data;
		
		//Accelerometer X
		//Read data and parse into one variable
		Ret = Serial1.readByte(&in_byte, 0);
		E_accel_x_data = in_byte;  //MSB
		Ret = Serial1.readByte(&in_byte, 0);

		//Convert data to engineering units
		E_accel_x_data = (E_accel_x_data << 8) | in_byte;  //LSB
		E_accel_x = 0.125*E_accel_x_data;
		
		//Accelerometer Y
		//Read data and parse into one variable
		Ret = Serial1.readByte(&in_byte, 0);
		E_accel_y_data = in_byte;  //MSB
		Ret = Serial1.readByte(&in_byte, 0);

		//Convert data to engineering units
		E_accel_y_data = (E_accel_y_data << 8) | in_byte;  //LSB
		E_accel_y = 0.125*E_accel_y_data;
		
		//Accelerometer Z
		//Read data and parse into one variable
		Serial1.readByte(&in_byte, 0);
		E_accel_z_data = in_byte;  //MSB
		Serial1.readByte(&in_byte, 0);

		//Convert data to engineering units
		E_accel_z_data = (E_accel_z_data << 8) | in_byte;  //LSB
		E_accel_z = 0.125*E_accel_z_data;
		
		//GPIO (Data Ready)
		//Read data and parse into one variable
		Serial1.readByte(&in_byte, 0);
		E_gpio_data = in_byte;  //MSB
		Serial1.readByte(&in_byte, 0);
		E_gpio_data = (E_gpio_data << 8) | in_byte;  //LSB
    
		//Check last byte for 0x0D
    	Serial1.readByte(&in_byte, 0);

		if (in_byte == 0x0D)
			return 1;
      
  	}//end if
  
  	return 0;
}//end readBurstData()


int setEpsonIMU(const char *Device,const unsigned int Bauds)
{
	Ret=Serial1.Open(Device, Bauds);	// Open serial link at 230400 bauds
    if (Ret!=1) {	// If an error occured...
        printf ("Error while opening port. Permission problem ?\n");  
        return Ret;  // ... quit the application
    }
    printf ("Serial port opened successfully: %s\n", Device);
    Serial1.FlushReceiver();
  
  	//Initialize Data Ready pin
  	printf("Setting GPIO0_7.\n");
	gpio_export(EPSON_DATA_RDY_PIN);
	gpio_set_dir(EPSON_DATA_RDY_PIN, INPUT_PIN);
	usleep(1000000);

	//Ping register and wait for startup packet
  	while (checkStartUp() != 0)
    	printf(".");
  	printf("\n");
  
  	if(checkStatus() != 0)
    	printf("IMU is faulty!!\n");

	//Initialize Settings
	configurationMode();	//go into configuration mode
	setSamplesPerSec(0x07);	//set samples per second
	setTAP(0x07);	//set number of Taps
	manualMode();	//go into manual mode
	samplingMode();
	
	return 1;
}//end setEpsonIMU


int readEpsonIMU()
{
	//Check Data Ready pin to see if data is updated then read
	gpio_get_value(EPSON_DATA_RDY_PIN, &data_rdy_value);
	if (data_rdy_value)
	{
		if(readBurstData())
			return 1;
		else
			return 0;
	}//end if
	
	return 0;
}//end readEpsonIMU()


void printEpsonIMU()
{
	//printf("Flag:	0x%x\n", flag);
	printf("Epson IMU:\n");
	printf("Temp =		%f 	[C] 	0x%x\n", E_temperature, E_temperature_data);
	printf("Gyro X = 	%f	[deg/s]	0x%x\n", E_gyro_x, E_gyro_x_data);
	printf("Gyro Y = 	%f 	[deg/s]	0x%x\n", E_gyro_y, E_gyro_y_data);
	printf("Gyro Z = 	%f 	[deg/s]	0x%x\n", E_gyro_z, E_gyro_x_data);
	printf("Accel X = 	%f 	[mG]	0x%x\n", E_accel_x, E_accel_x_data);
	printf("Accel Y = 	%f 	[mG]	0x%x\n", E_accel_y, E_accel_y_data);
	printf("Accel Z = 	%f 	[mG]	0x%x\n\n", E_accel_z, E_accel_z_data);
	
	return;
}//end printEpsonIMU()


/*
 *	Get() Functions
 */
float getEpsonTemperature()
{
	return E_temperature;
}//end getEpsonAccelX()

float getEpsonGyroX()
{
	return E_gyro_x;
}//end getEpsonGyroX()

float getEpsonGyroY()
{
	return E_gyro_y;
}//end getEpsonGyroY()

float getEpsonGyroZ()
{
	return E_gyro_z;
}//end getEpsonGyroZ()

float getEpsonAccelX()
{
	return E_accel_x;
}//end getEpsonAccelX()

float getEpsonAccelY()
{
	return E_accel_y;
}//end getEpsonAccelY()

float getEpsonAccelZ()
{
	return E_accel_z;
}//end getEpsonAccelZ()








