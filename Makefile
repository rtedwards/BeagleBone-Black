
#Libraries are stored in the following subdirectories
ANA=Libraries/AnalogIMU
ADC=Libraries/BBB_ADC
EPS=Libraries/EpsonIMU
SIM=Libraries/SimpleGPIO
SER=Libraries/serialib
BACK=../..

BBBB: main.o serialib.o BBB_ADC.o AnalogIMU.o EpsonIMU.o SimpleGPIO.o
	g++ -lncurses -o BBBB main.o serialib.o BBB_ADC.o AnalogIMU.o EpsonIMU.o SimpleGPIO.o
	
main.o: main.c $(ANA)/AnalogIMU.h $(EPS)/EpsonIMU.h $(ADC)/BBB_ADC.h $(SER)/serialib.h $(SIM)/SimpleGPIO.h
	g++ -c main.c

serialib.o: $(SER)/serialib.cpp $(SER)/serialib.h
	g++ -c $(SER)/serialib.cpp
	
BBB_ADC.o: $(ADC)/BBB_ADC.c $(ADC)/BBB_ADC.h
	gcc -c $(ADC)/BBB_ADC.c
	
AnalogIMU.o: $(ANA)/AnalogIMU.c $(ANA)/AnalogIMU.h
	gcc -c $(ANA)/AnalogIMU.c
	
SimpleGPIO.o: $(SIM)/SimpleGPIO.cpp $(SIM)/SimpleGPIO.h
	g++ -c $(SIM)/SimpleGPIO.cpp

EpsonIMU.o: $(EPS)/EpsonIMU.cpp $(EPS)/EpsonIMU.h $(SER)/serialib.h $(SIM)/SimpleGPIO.h
	g++ -c $(EPS)/EpsonIMU.cpp 

### Starting Program ###
ADT_STARTUP:	ADT_StartUp.o 
	g++ -o ADT_STARTUP ADT_StartUp.o 

ADT_StartUp.o:	ADT_StartUp.c
	g++ -c ADT_StartUp.c


### Remote Starting Program ###
REMOTE: Remote_Start.o serialib.o
	g++ -o REMOTE Remote_Start.o $(SER)/serialib.o

Remote_Start.o: Remote_Start.c $(SER)/serialib.h
	g++ -c Remote_Start.c 


### Clean up *.o and executables ###
clean:
	rm *.o BBBB ADT_STARTUP

