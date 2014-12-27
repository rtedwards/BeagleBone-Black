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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "AnalogIMU.h"

static uint8_t mode = SPI_MODE_3;
static uint8_t bits = 16;
static uint32_t speed = 1000000;
static uint16_t delay;

//Register Assignments
static uint16_t prod_ID_reg = 0x7E00;	//Product ID
static uint16_t temperature_reg = 0x0E00;
static uint16_t gyro_x_reg = 0x1200;
static uint16_t gyro_y_reg = 0x1600;
static uint16_t gyro_z_reg = 0x1A00;
static uint16_t accel_x_reg = 0x1E00;
static uint16_t accel_y_reg = 0x2200;
static uint16_t accel_z_reg = 0x2600;
static uint16_t magnet_x_reg = 0x2800;
static uint16_t magnet_y_reg = 0x2A00;
static uint16_t magnet_z_reg = 0x2C00;	
static uint16_t barometer_reg = 0x3000;


//***** Functions *****//
//Abort Function: prints error to screen and aborts program
static void pabort(const char *s)
{
	perror(s);
	abort();
}//end pabort()


/*transfer(): Does SPI transfer of data from tx[] buffer and reads
data from device into rx[] buffer.  Data will be in rx[1] slot.  
The first slot of tx[] buffer is the uint16_t value to be sent to SPI device
followed by 0x0000, 0x0000, 0x0000 to allow time for transfer and to receive
data back.
*/
static void transfer(int fd, uint16_t *tx, uint16_t *rx, uint8_t size, uint8_t print_flag)
{
	int ret;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = size,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};


	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);	//SPI transfer
	if (ret < 1)
		pabort("can't send spi message");

	if (print_flag) {	//Will print data if told to via print_flag
		for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
			if (!(ret % 6))
				puts("");
			printf("%.2X ", rx[ret]);
		}
		puts("");
	}//end if
}//end transfer()

//Begin communications with specified SPI device
int openAnalogSPI(const char *device)
{
	int fd;

	fd = open(device, O_RDWR);
	if (fd < 0){
		pabort("can't open device");
		return -1;
	}//end if

	return fd;
}//end openAnaloSPI()

//Will close communications with specified SPI device
void closeAnalogSPI(int fd)
{
	close(fd);
	return;
}//end closeSPI()

/*Configures SPI commmunications paramters in the Linux ioctl()
* Mode: 3
* Bits per word: 16 (ADIS16480 need a 16 bit word per transfer
* Speed: 1,000,000
*/
void setAnalogSPI(int fd)
{
	int ret;
	
	//spi write mode
	printf("SPI Mode: %d\n", mode);
	
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("Can't set spi mode");
	
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
	else
		printf("SPI Mode: %d\n", mode);
		

	//bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");


	//max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d MHz)\n", speed, speed/1000/1000);

	return;
}//end setSPI()

/* Configures ADIS16480 from default startup settings.  
* Nmaely from North-East-Down frame to Body frame coordinates
*/
int setAnalogIMU(const char *device)
{
	int fd = openAnalogSPI(device);	//Open BBB SPI port
	setAnalogSPI(fd);	//Set SPI Port
	
	
	/* SETTINGS
	 * 	Automatic reset recovery from divergence
	 * 	Fade enabled
	 *	Body Frame
	 */
	uint16_t temp = readRegister(fd, 0x5100);
	printf("reg 0x51: %x\n", temp);
	temp = readRegister(fd, 0x5000);
	printf("reg 0x50: %x\n", temp);
	
	while(temp != 0x1208)
	{
		writeRegister(fd, 0x0003);	//Set to PG_ID 3
		writeRegister(fd, 0x5112);	//Fade enabled
		writeRegister(fd, 0x5008);	//Body frame
	
		temp = readRegister(fd, 0x5100);
		printf("reg 0x51: %x\n", temp);
		temp = readRegister(fd, 0x5000);
		printf("reg 0x50: %x\n", temp);
	}//end while
	
	writeRegister(fd, 0x0000);	//Set to PG_ID 0

	return fd;
}//end setIMU()


/*Gets data from Gyroscope, Accelerometers, Magnetometers, Temperature sensor,
* and Barometer then converts to engineering units.
*/
void readAnalogIMU(int fd)
{	
		A_temperature = readRegister(fd, temperature_reg) * 0.00565 + 25; //Temperature [C]
		A_gyro_x = readRegister(fd, gyro_x_reg) * 0.02; //GyroX [deg/s]
		A_gyro_y = readRegister(fd, gyro_y_reg) * 0.02; //GyroY [deg/s]
		A_gyro_z = readRegister(fd, gyro_z_reg) * 0.02; //GyroZ [deg/s]
		A_accel_x = readRegister(fd, accel_x_reg) * 0.8; //AccelX [milli g]
		A_accel_y = readRegister(fd, accel_y_reg) * 0.8; //AccelY [milli g]
		A_accel_z = readRegister(fd, accel_z_reg) * 0.8; //AccelZ [milli g]
		A_magnet_x = readRegister(fd, magnet_x_reg) * 0.1; //Magnetometer X [mGauss]
		A_magnet_y = readRegister(fd, magnet_y_reg) * 0.1; //Magnetometer Y [mGauss]
		A_magnet_z = readRegister(fd, magnet_z_reg) * 0.1; //Magnetometer Z [mGauss]
		A_barometer = readRegister(fd, barometer_reg) * 40; //Barometer [mBar]
	return;	//data not ready or error reading
}//end readAnalogIMU()

// Prints the data last read from the ADIS16480 to the terminal
void printAnalogIMU()
{
	printf("Analog IMU:\n");
	printf("Temp = 		%f 	[C]\n", A_temperature);
	printf("Gyro X = 	%f 	[deg/s]\n", A_gyro_x);
	printf("Gyro Y = 	%f 	[deg/s]\n", A_gyro_y);
	printf("Gyro Z = 	%f 	[deg/s]\n", A_gyro_z);
	printf("Accel X = 	%f 	[mg]\n", A_accel_x);
	printf("Accel Y = 	%f 	[mg]\n", A_accel_y);
	printf("Accel Z = 	%f 	[mg]\n", A_accel_z);
	printf("Mag X = 	%f 	[mG]\n", A_magnet_x);
	printf("Mag Y = 	%f 	[mG]\n", A_magnet_y);
	printf("Mag Z = 	%f 	[mG]\n", A_magnet_z);
	printf("Barom = 	%f 	[mBar]\n\n", A_barometer);
	
	return;
}//end printAnalogIMU()

/*Writes data to the specified SPI device in the format
* [value, 0x0000, 0x0000, 0x0000] 
* Will also set the Most Significant Bit so the ADIS16480 knows
* this is a write operation and not a read operation.
*/
void writeRegister(int fd, uint16_t value)
{
	value = value | 0x8000;	//MSB set to "1" for "write"
	uint16_t tx_write[] = {value, 0x0000, 0x0000, 0x0000};	
	uint16_t rx_write[ARRAY_SIZE(tx_write)] = {0, };	
	transfer(fd, tx_write, rx_write, ARRAY_SIZE(tx_write), 1);
	
	return;
}//end writeRegister()

/*Writes data to the specified SPI device in the format
* [value, 0x0000, 0x0000, 0x0000] 
* Will also set the Most Significant Bit so the ADIS16480 knows
*/
int16_t readRegister(int fd, uint16_t value)
{
	uint16_t tx_read[] = {value, 0x0000, 0x0000, 0x0000};	
	uint16_t rx_read[ARRAY_SIZE(tx_read)] = {0, };	
	transfer(fd, tx_read, rx_read, ARRAY_SIZE(tx_read), 0);
	
	return rx_read[1];
}//end readRegister()


//***** Get Functions *****//
float getAnalogTemperature()
{
	return A_temperature;
}//end getAnalogTemperature()

float getAnalogBarometer()
{
	return A_barometer;
}//end getAnalogBarometer()

float getAnalogGyroX()
{
	return A_gyro_x;
}//end getAnalogGyroX()

float getAnalogGyroY()
{
	return A_gyro_y;
}//end getAnalogGyroY()

float getAnalogGyroZ()
{
	return A_gyro_z;
}//end getAnalogGyroZ()

float getAnalogAccelX()
{
	return A_accel_x;
}//end getAnalogAccelX()

float getAnalogAccelY()
{
	return A_accel_y;
}//end getAnalogAccelY()

float getAnalogAccelZ()
{
	return A_accel_z;
}//end getAnalogAccelZ()

float getAnalogMagnetX()
{
	return A_magnet_x;
}//end getAnalogMagnetX()

float getAnalogMagnetY()
{
	return A_magnet_y;
}//end getAnalogMagnetY()

float getAnalogMagnetZ()
{
	return A_magnet_z;
}//end getAnalogMagnetZ()



