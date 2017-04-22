#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000
#include <util/delay.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define RING_BUFFER_SIZE 128
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//EEPROM variables for motor settings
uint16_t motorIntLen EEMEM;
int16_t motorDifInt EEMEM;
uint8_t driveMotorsEEPROM[128][3] EEMEM;

uint8_t driveMotors[128][3];
uint8_t driveIRCountsToShutdown[30];

struct ringBuffer{
	char ring[RING_BUFFER_SIZE];
	uint8_t read;
	uint8_t write;
} bufferUART;

void sleep ( uint8_t ms );
void printchr (char string);
void help(void);
void prepare_pins (void);
void prepare_timer();
void test_pins(volatile uint8_t *pport, uint8_t min, uint8_t max);
void test_loop (void);
void switchMotorOn(uint8_t pinID);
void switchMotorOff(uint8_t pinID);
void decreaseMotorTimers(void);
void initializeEEPROM(void);
void showMotorMap();
void changeMotorMap();
uint8_t getNumber();
uint8_t programState; //bit1: decrease motor time
