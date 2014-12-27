/*
	Author: Robert Edwards

	This library is made to be used with the BeagleBone Black running 
	Linux Angstrom 3.8.13.  	

	Enables use of the 7 12-bit Analog to Digital Converters (ADC) on the BeagleBone 
	Black.  Available ADCs are:
	
	0 - P9 pin# 39
	1 - P9 pin# 40
	2 - P9 pin# 37
	3 - P9 pin# 38
	4 - P9 pin# 33
	5 - P9 pin# 36
	6 - P9 pin# 35
	ADC GND - P9 pin# 34
	ADC Vdd - P9 pin# 32

	ADC Properties
	-12 bit (0-4095) 
	-0-1.8V range
	-125 ns sample time
	-2 micro Amps
	-Voltage Divider - leg connected to ground should be <= 1 kOhm
	-Voltage Divider - use resistors with a 0.1% error tolerance
	-ADC GND should be grounded

	Enable driver with "echo BB-ADC > /sys/devices/bone_capemgr.8/slots"
	or with "echo cape-bone-iio > /sys/devices/bone_capemgr.8/slots"
 */

#ifdef __cplusplus
extern "C"{
#endif

#ifndef BBB_ADC_H_
#define BBB_ADC_H_


#define MAX_BUF 64
int readADC(unsigned int pin);

#endif

#ifdef __cplusplus
}
#endif
