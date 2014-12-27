/*
 *  Author:  Robert Edwards
 * 
 *	Will read data from Epson IMU, Analog Devices IMU, and voltage 
 *	from makeshift sunsensor and save to a file.
 */


#include <stdio.h>
#include <sys/time.h>
#include "Libraries/serialib/serialib.h"
#include "Libraries/EpsonIMU/EpsonIMU.h"
#include "Libraries/SimpleGPIO/SimpleGPIO.h"
#include <ncurses.h>

//These files were compiled in C rather than C++
extern "C" {

#include "Libraries/AnalogIMU/AnalogIMU.h"
#include "Libraries/BBB_ADC/BBB_ADC.h"

}//end extern "C"

#define XBEE "/dev/ttyUSB0"
//#define XBEE "/dev/ttyO4"
#define EPSON "/dev/ttyO1"
#define ANALOG_DEVICES "/dev/spidev1.0"
#define MAX_PATH_BUF 50 	//length of path buffer
#define PACKET_SIZE 84	//Number of bytes in telemetry packet
#define ANALOG_DATA_RDY_PIN 20	//GPIO_20 pin 40


/**************************************************
					FUNCTIONS
**************************************************/
FILE * openSaveFile();
void sendFloat(float value);
void sendUShort(uint16_t value);
void sendData();
uint16_t CalculateCheckSum(uint8_t *packet);


/**************************************************
					VARIABLES
**************************************************/
serialib Serial;	//create serialib object
unsigned int LED_RED = 26;	//Status LED: XBee
unsigned int LED_GREEN = 44;	//Status LED: Analog IMU
unsigned int LED_BLUE = 68;	//Status LED: Epson IMU
unsigned int adc0;
unsigned int adc1;
unsigned int adc2;
unsigned int adc3;
unsigned int Analog_data_rdy = 0;
unsigned int Epson_data_rdy = 0;
uint16_t checksum = 0;
uint8_t packet[84];	//Packet to be sent to Matlab

//Setup for timing
timeval time0, time1, time2, time3;	//Time variables
double elapsed_time, elapsed_time_old, data_time, split_time;
	
union TranfserFloat
{//used to send float values over Serial Port
	long l;
	float f;
} transfer_float;	//end TransferFloat



/**************************************************
					MAIN()
**************************************************/
int main(void)
{
	//***** INITIALIZATIONS *****//
	//*****************************

	//***** GPIO Pins Setup *****//
	//Status LEDs Setup
	printf("Setting up Status LEDs\n");
	gpio_export(LED_RED);
	gpio_export(LED_GREEN);
	gpio_export(LED_BLUE);
	printf("Status LED Exported\n");
	gpio_set_dir(LED_RED, OUTPUT_PIN);
	gpio_set_dir(LED_GREEN, OUTPUT_PIN);
	gpio_set_dir(LED_BLUE, OUTPUT_PIN);
	printf("Status LED Direction set\n");

	//Flash LED to show program started
	for (int i=0;i<5;i++)
	{
		printf("Status LED: %d\n", i);
		gpio_set_value(LED_RED, HIGH);
		usleep(250000);		//pause for 125,000 micro seconds
		gpio_set_value(LED_RED, LOW);
		
		gpio_set_value(LED_GREEN, HIGH);
		usleep(250000);		//pause for 125,000 micro seconds
		gpio_set_value(LED_GREEN, LOW);
		
		gpio_set_value(LED_BLUE, HIGH);
		usleep(250000);		//pause for 125,000 micro seconds
		gpio_set_value(LED_BLUE, LOW);
	}//end while

	//***** Analog Devices ADIS16480 Setup *****//
	gpio_set_value(LED_GREEN, HIGH);	//GREEN LED on during Analog setup
	
	//Analog Devices Data Ready pin (GPIO_20)
  	printf("Setting GPIO0_20.\n");
	gpio_export(ANALOG_DATA_RDY_PIN);
	gpio_set_dir(ANALOG_DATA_RDY_PIN, INPUT_PIN);
	
	printf("\nInitializing Analog Devices IMU\n");
	//int spi_port = openAnalogSPI(ANALOG_DEVICES);
	//setAnalogSPI(spi_port);		//configures spi port setting
	int spi_port = setAnalogIMU(ANALOG_DEVICES);		//configures IMU settings
	gpio_set_value(LED_GREEN, LOW);	//GREEN LED on during Analog setup
	
	
	//***** Serial Port Initializations *****//
	gpio_set_value(LED_RED, HIGH);	//RED LED on during XBee setup
	printf("\nOpening XBee Serial port\n");
	int ret;	//return state for fclose()
	while (ret != 1)
	{
		ret = Serial.Open(XBEE,38400);	// Open serial link at 38400 bauds
		if (ret != 1)
		{
			printf("Could not open port: %d\n", ret);
		}//end if
	}//end while
	gpio_set_value(LED_RED, LOW);	//RED LED on during XBee setup
	
	
	//***** Epson IMU Setup *****//
	gpio_set_value(LED_BLUE, HIGH);	//BLUE LED on during Epson setup
	
	printf("\nInitializing Epson IMU\n");
	setEpsonIMU(EPSON, 230400);
	gpio_set_value(LED_BLUE, LOW);	//BLUE LED on during Epson setup
	
	
	//***** Initialize Save File .csv *****//
	FILE *file = openSaveFile();

	gettimeofday(&time0, NULL);	//Initial time
	gettimeofday(&time1, NULL);	//Record total time elapsed
				
	elapsed_time = (double)(time1.tv_sec - time0.tv_sec) * 1000.0;	//sec to ms				
	elapsed_time += (double)(time1.tv_usec - time0.tv_usec) / 1000.0;	//us to ms


	//***** CONTROL LOOP *****//
	int count = 0;
	//initscr();	//initialize ncurses screen	
	while(1){	
		elapsed_time_old = elapsed_time;	//set beginning time of last loop
		gettimeofday(&time1, NULL);	//Record total time elapsed	
		
		//Flash LED WHITE every other loop to show program is running
		if (count % 2 == 0)
		{
			gpio_set_value(LED_RED, HIGH);	//
			gpio_set_value(LED_GREEN, HIGH);//
			gpio_set_value(LED_BLUE, HIGH);	//
		}//end if
		else
		{
			gpio_set_value(LED_RED, LOW);	//
			gpio_set_value(LED_GREEN, LOW);	//
			gpio_set_value(LED_BLUE, LOW);	//
		}//end else
		count++;
		
		
		//set beginning time of current loop
		elapsed_time = (double)(time1.tv_sec - time0.tv_sec) * 1000.0;	//sec to ms				
		elapsed_time += (double)(time1.tv_usec - time0.tv_usec) / 1000.0;	//us to ms
		
		split_time = elapsed_time - elapsed_time_old;	//Calculate time to run through last loop
		
		//printf("Elapsed Time: 	%f\n", elapsed_time);
		//printf("Split Time:	%f\n", split_time);
		//printf("Data Time: 	%f\n\n", data_time);
		
		//Read Epson IMU
		readEpsonIMU();
		//printEpsonIMU();
		
		//Read Analog Devices IMU
		gpio_get_value(ANALOG_DATA_RDY_PIN, &Analog_data_rdy);
		if (Analog_data_rdy)
		{
			readAnalogIMU(spi_port);
			//printAnalogIMU();
		}//end if
		
		//Read Sun Sensor values
		adc0 = readADC(0);
		adc1 = readADC(1);
		adc2 = readADC(2);
		adc3 = readADC(3);
		
		//Calculate time it took to receive data
		gettimeofday(&time2, NULL);	//Record total time elapsed
		data_time = (double)(time2.tv_sec - time1.tv_sec) * 1000.0;	//sec to ms				
		data_time += (double)(time2.tv_usec - time1.tv_usec) / 1000.0;	//us to ms
		
		//Save data to .csv file
		fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d\n",elapsed_time, getEpsonTemperature(), getEpsonGyroX(), getEpsonGyroY(), getEpsonGyroZ(), getEpsonAccelX(), getEpsonAccelY(), getEpsonAccelZ(), getAnalogTemperature(), getAnalogGyroX(), getAnalogGyroY(), getAnalogGyroZ(), getAnalogAccelX(), getAnalogAccelY(), getAnalogAccelZ(), getAnalogMagnetX(), getAnalogMagnetY(), getAnalogMagnetZ(), adc0, adc1, adc2, adc3);
		
		//Send data to remote computer over XBEE
		sendData();	
		
		/*** Print Data to Screen (NCurses)  ***/
/*		clear();	//NCurses: erase screen
		printw("Elapsed Time:	%f	[ms] (Time since program start)\n", elapsed_time);
		printw("Split Time:	%f	[ms] (Time of previous loop)\n", split_time);
		printw("Data Time:	%f	[ms] (Time to collect all data)\n\n", data_time);
		printw("A Barometer:	%f	[microBar]\n\n", getAnalogBarometer());
		printw("A Temp	=	%f	[C]\n",getAnalogTemperature());
		printw("E Temp	=	%f	[C]\n\n",getEpsonTemperature());
		printw("A Gyro	X: %f	Y: %f	Z: %f	[deg/s]\n",getAnalogGyroX(), getAnalogGyroY(), getAnalogGyroZ());
		printw("E Gyro	X: %f	Y: %f	Z: %f	[deg/s]\n\n", getEpsonGyroX(), getEpsonGyroY(), getEpsonGyroZ());
		printw("A Accel	X: %f	Y: %f	Z: %f	[milli g]\n",getAnalogAccelX(), getAnalogAccelY(), getAnalogAccelZ());
		printw("E Accel	X: %f	Y: %f	Z: %f	[milli g]\n\n", getEpsonAccelX(), getEpsonAccelY(), getEpsonAccelZ());
		printw("A Mag	X: %f	Y: %f	Z: %f	[mGauss]\n\n",getAnalogMagnetX(), getAnalogMagnetY(), getAnalogMagnetZ());
		printw("Sun	1: %d	2: %d	3: %d	4: %d	[mV]\n\n", adc0, adc1, adc2, adc3);
*/		refresh();	//NCurses: print to screen
	
		
		//Calculate time it took to receive data
		gettimeofday(&time2, NULL);	//Record total time elapsed
		data_time = (double)(time2.tv_sec - time1.tv_sec) * 1000.0;	//sec to ms				
		data_time += (double)(time2.tv_usec - time1.tv_usec) / 1000.0;	//us to ms
		
		usleep(100000 - data_time*1000.0);	//sleep exactly 100ms (10Hz)

	}//end while
	
	fclose(file);	//Close Save file
	closeAnalogSPI(spi_port);	//Close SPI port
	Serial.Close();	//close XBee serial port
	//endwin();	//closes ncurses screen	

	return 0;
}//end main()



FILE * openSaveFile()
{
	//***** Opening Files for logging data *****//
	printf("\nOpening Data File Log\n");
	FILE *file_log = fopen("/home/xuser/BBBB/Data_Files/Data_Log.txt", "a+");
	if (file_log == NULL)
		printf("Error opening Log File\n");
		
	
	//Read file log number
	int log_number = 0;
	while (!feof (file_log))
	{
		fscanf(file_log, "%d,", &log_number);
		printf("File Log: %d \n", log_number);
	}//end while
	fclose(file_log);	//close file
	
	file_log = fopen("/home/xuser/BBBB/Data_Files/Data_Log.txt", "w");	//Delete/clear file
	log_number = log_number + 1;
	fprintf(file_log, "%d", log_number);
	fclose(file_log);
	
	char path_buf[MAX_PATH_BUF];
	snprintf(path_buf, sizeof(path_buf), "/home/xuser/BBBB/Data_Files/BBBB_Data_%d.csv", log_number);
	
	//Open file for saving data
	printf("\nOpening data file\n");
	FILE *file  = fopen(path_buf, "w");
	if (file == NULL) {
		printf("Error opening file!\n");
		exit(1);
	}//end if
	setlinebuf(file);	//Will now update file after every line
	
	return file;
}//end openSaveFile()

void sendFloat(float value)
{
	uint8_t send_byte;
	transfer_float.f = value;	//copy float value into union float
	
	send_byte = transfer_float.l | 0x00;
	Serial.writeByte(send_byte);	//Send Least significant bit
	send_byte = (transfer_float.l >> 8) | 0x00;
	Serial.writeByte(send_byte);
	send_byte = (transfer_float.l >> 16) | 0x00;
	Serial.writeByte(send_byte);
	send_byte = (transfer_float.l >> 24) | 0x00;
	Serial.writeByte(send_byte);	//Send Most significant bit
	
	return;
}//end sendFloat()


void sendUShort(uint16_t value)
{
	uint8_t send_byte;
	
	send_byte = value | 0x00;
	Serial.writeByte(send_byte);	//Send Lest Significant Byte
	send_byte = (value >> 8) | 0x00;
	Serial.writeByte(send_byte);	//Send Most Significant Byte
	
	return;
}//end sendFloat()


uint16_t CalculateCheckSum(uint8_t *packet)
{
	uint16_t checksum = 0;

	packet[0] = 0xEB;
	packet[1] = 0x90;

	transfer_float.f = elapsed_time;
	packet[2] = (transfer_float.l >> 24) | 0x00;
	packet[3] = (transfer_float.l >> 16) | 0x00;
	packet[4] = (transfer_float.l >> 8) | 0x00;
	packet[5] = transfer_float.l | 0x00;

	transfer_float.f = getEpsonTemperature();
	packet[6] = (transfer_float.l >> 24) | 0x00;
	packet[7] = (transfer_float.l >> 16) | 0x00;
	packet[8] = (transfer_float.l >> 8) | 0x00;
	packet[9] = transfer_float.l | 0x00;

	transfer_float.f = getEpsonGyroX();
	packet[10] = (transfer_float.l >> 24) | 0x00;
	packet[11] = (transfer_float.l >> 16) | 0x00;
	packet[12] = (transfer_float.l >> 8) | 0x00;
	packet[13] = transfer_float.l | 0x00;

	transfer_float.f = getEpsonGyroY();
	packet[14] = (transfer_float.l >> 24) | 0x00;
	packet[15] = (transfer_float.l >> 16) | 0x00;
	packet[16] = (transfer_float.l >> 8) | 0x00;
	packet[17] = transfer_float.l | 0x00;

	transfer_float.f = getEpsonGyroZ();
	packet[18] = (transfer_float.l >> 24) | 0x00;
	packet[19] = (transfer_float.l >> 16) | 0x00;
	packet[20] = (transfer_float.l >> 8) | 0x00;
	packet[21] = transfer_float.l | 0x00;

	transfer_float.f = getEpsonAccelX();
	packet[22] = (transfer_float.l >> 24) | 0x00;
	packet[23] = (transfer_float.l >> 16) | 0x00;
	packet[24] = (transfer_float.l >> 8) | 0x00;
	packet[25] = transfer_float.l | 0x00;

	transfer_float.f = getEpsonAccelY();
	packet[26] = (transfer_float.l >> 24) | 0x00;
	packet[27] = (transfer_float.l >> 16) | 0x00;
	packet[28] = (transfer_float.l >> 8) | 0x00;
	packet[29] = transfer_float.l | 0x00;

	transfer_float.f = getEpsonAccelZ();
	packet[30] = (transfer_float.l >> 24) | 0x00;
	packet[31] = (transfer_float.l >> 16) | 0x00;
	packet[32] = (transfer_float.l >> 8) | 0x00;
	packet[33] = transfer_float.l | 0x00;

	transfer_float.f =  getAnalogTemperature();
	packet[34] = (transfer_float.l >> 24) | 0x00;
	packet[35] = (transfer_float.l >> 16) | 0x00;
	packet[36] = (transfer_float.l >> 8) | 0x00;
	packet[37] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogGyroX();
	packet[38] = (transfer_float.l >> 24) | 0x00;
	packet[39] = (transfer_float.l >> 16) | 0x00;
	packet[40] = (transfer_float.l >> 8) | 0x00;
	packet[41] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogGyroY();
	packet[42] = (transfer_float.l >> 24) | 0x00;
	packet[43] = (transfer_float.l >> 16) | 0x00;
	packet[44] = (transfer_float.l >> 8) | 0x00;
	packet[45] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogGyroZ();
	packet[46] = (transfer_float.l >> 24) | 0x00;
	packet[47] = (transfer_float.l >> 16) | 0x00;
	packet[48] = (transfer_float.l >> 8) | 0x00;
	packet[49] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogAccelX();
	packet[50] = (transfer_float.l >> 24) | 0x00;
	packet[51] = (transfer_float.l >> 16) | 0x00;
	packet[52] = (transfer_float.l >> 8) | 0x00;
	packet[53] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogAccelY();
	packet[54] = (transfer_float.l >> 24) | 0x00;
	packet[55] = (transfer_float.l >> 16) | 0x00;
	packet[56] = (transfer_float.l >> 8) | 0x00;
	packet[57] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogAccelZ();
	packet[58] = (transfer_float.l >> 24) | 0x00;
	packet[59] = (transfer_float.l >> 16) | 0x00;
	packet[60] = (transfer_float.l >> 8) | 0x00;
	packet[61] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogMagnetX();
	packet[62] = (transfer_float.l >> 24) | 0x00;
	packet[63] = (transfer_float.l >> 16) | 0x00;
	packet[64] = (transfer_float.l >> 8) | 0x00;
	packet[65] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogMagnetY();
	packet[66] = (transfer_float.l >> 24) | 0x00;
	packet[67] = (transfer_float.l >> 16) | 0x00;
	packet[68] = (transfer_float.l >> 8) | 0x00;
	packet[69] = transfer_float.l | 0x00;

	transfer_float.f = getAnalogMagnetZ();
	packet[70] = (transfer_float.l >> 24) | 0x00;
	packet[71] = (transfer_float.l >> 16) | 0x00;
	packet[72] = (transfer_float.l >> 8) | 0x00;
	packet[73] = transfer_float.l | 0x00;

	packet[74] = (adc0 >> 8) | 0x00;
	packet[75] = adc0 | 0x00;
	packet[76] = (adc1 >> 8) | 0x00;
	packet[77] = adc1 | 0x00;
	packet[78] = (adc2 >> 8) | 0x00;
	packet[79] = adc2 | 0x00;
	packet[80] = (adc3 >> 8) | 0x00;
	packet[81] = adc3 | 0x00;

	for (int i=0; i<PACKET_SIZE - 2; i++) 
	{//Sum individual bytes of packet excluding the checksum
		checksum += packet[i];
	}//end checkSum

	//Include checksum to end of packet
	packet[82] = checksum | 0x00;
	packet[83] = (checksum >> 8) | 0x00;

	return checksum;
}//end CalculateCheckSum()



void sendData()
{
	//Calculate checksum
	checksum = CalculateCheckSum(packet);

	for (int i=0; i<84; i++)
		Serial.writeByte(packet[i]);
		
	return;
}//end sendData()







