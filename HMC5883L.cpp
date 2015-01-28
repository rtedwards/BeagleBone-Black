/*
	Author: Robert Edwards
	
	BeagleBone Black driver for HMC5883L magnetometer.
	Communication to magnetometer is I2C protocal using
	the i2c-1 port on the BeagleBone Black.  
	I2C2_SCL - pin 19 P9
	I2C2_SDA - pin 20 P9
	Linux port is i2c-1 but pinout is I2C2 (19,20)
*/
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define DELAY 5*1000*100

int main()
{
	//Opening Bus
	int file;
	char *filename = "/dev/i2c-1";
	if ((file = open(filename, O_RDWR)) < 0) {
		//opening i2c port of BeagleBone Black
		perror("Failed to open the i2c bus");
		exit(1);
	}//end if

	//Initiating Comms
	int addr = 0x1E; //I2C address of HMC5883L
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		//Identifying HMC5883L device on i2c line
		printf("Failed to acquire buss access and/or talk to slave. \n");
		exit(1);
	}//end if


	/*Writing to HMC5883L*/
	//Device register to access
	char read_buffer[13] = {0};	//Stores vales read from registers (13 registers)
	char rd_wr_buffer[2] = {0};	//Data to be written to device. 2 bytes (port, value)
	char reg_buffer[1] = {0};	//Value of first register
	unsigned char register_value = 0x00; //Value of first register

	/*Writing address to read from*/
	//Configuration Register A
	rd_wr_buffer[0] = 0x00;	//Register 0x00
	rd_wr_buffer[1] = 0x70;	//Configuration value (read DataSheet)

	puts("Setting Registers A, B, Mode");
	if (write(file, rd_wr_buffer, 2) != 2) {
		//Write 1 byte from reg_buffer to HMC5883L and check it was written
		printf("Failed to write to I2C bus.\n\n");
	}//end if
	else
		printf("0x%d to 0x%d\n", rd_wr_buffer[0], rd_wr_buffer[1]);

	//Configuration Register B
	rd_wr_buffer[0] = 0x01;	//Register 0x01
	rd_wr_buffer[1] = 0xA0;	//Configuration value (read DataSheet)
	if (write(file, rd_wr_buffer, 2) != 2) {
		//Write 1 byte from reg_buffer to HMC5883L and check it was written
		printf("Failed to write to I2C bus.\n\n");
	}//end if
	else
		printf("0x%d to 0x%d\n", rd_wr_buffer[0], rd_wr_buffer[1]);

	//Mode Register
	rd_wr_buffer[0] = 0x02;	//Register 0x02
	rd_wr_buffer[1] = 0x00;	//Configuration value (read DataSheet)
	if (write(file, rd_wr_buffer, 2) != 2) {
		//Write 1 byte from reg_buffer to HMC5883L and check it was written
		printf("Failed to write to I2C bus.\n\n");
	}//end if
	else
		printf("0x%d to 0x%d\n", rd_wr_buffer[0], rd_wr_buffer[1]);

	usleep(6*1000); //sleep 6 ms


	/*Reading from HMC5883L*/
	printf("\n500 ms Delay in Readings.\n");
	puts("-------------------------");
	printf(" X  | Z |  Y \n");
	//Create short (2 bytes) for concatenating MSB and LSB
	short value = 0;	//concatenated calue
	int count = 0;		//Counter for alternating shifts


	//***** Main Loop *****//
	while (1) {
		//Reset To register 0x03
		reg_buffer[0] = register_value;	//Reset Register Ptr to first register
		if (write(file, reg_buffer, 1) != 1) {
			//Write 1 byte from reg_buffer to HMC5883L and check it was written
			printf("Failed to write to I2C bus.\n\n");
		}//end if

		//Using I2C read
		if (read(file, read_buffer, 13) != 13) {
			//Read 13 bytes into "read_buffer" then check 13 bytes were read
			//Internal HMC5883L Addr Ptr automatically incremented after every read 
			printf("Failed to read from the I2C bus: %s.\n", strerror(errno));
			printf("1. Check if wired to correct pins\n");
			printf("2. Check if the pins are set properly\n\n");
		}//end if
		else {
			//printf("Looks like the I2C bus is operational! \n");
			for (int j=3; j<9; j++) {
				value = value | read_buffer[j];	//OR read_buffer into lower byte
				if (count%2 == 0)	//Shift 8 bits every other loop		
					value = value << 8;
				else {
					printf("%d ",value);
					value = 0;
				}//end if
				count++;
			}// end for
			
			count = 0;
		}//end else
		
		printf("\n");
		usleep(DELAY); //500 ms
	}//end while()

	return 0;
}//end main()
