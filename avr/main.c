#include "main.h"

const char helpLine[25][50] PROGMEM = {
"######################",//0
"# Welcome to AVibeIn #",
"######################",
"",
"Options:",
"1) start input (end with ESC)",
"2) change character-to-motor assignment",
"3) change motor timings",
"4) test loop through all outputs",
"5) initialize / reset EEPROM",
"6) print motor map", //10
"",
"** Testloop (250ms per output) **",
"Please enter 3 motors, separate and end with ;",
"** change motor mapping **",
"select character to change", //15
"** printing motor map **",
"$$",
"PORTA",
"PORTB",
"PORTC",//20
"PORTD",
"** initializing EEPROM **",
"EEPROM written",
"waiting for input ..."
};

//generic sleep function
void sleep ( uint8_t ms )
{
    for(; ms > 0; ms--) _delay_ms(1);
}

//return char arrays with CR & LF
void println (char *string)
{
	int i = 0;
	while ( string[i]!='\0' )
	{
		while ((UCSRA & (1 << UDRE)) == 0) {}; 
		UDR = string[i];
		i++;
	}
	while ((UCSRA & (1 << UDRE)) == 0) {};
	UDR = 0x0D; //CR
	while ((UCSRA & (1 << UDRE)) == 0) {};
	UDR = 0x0A; //LF
}

//return char with CR & LF
void printchr (char string)
{
	while ((UCSRA & (1 << UDRE)) == 0) {}; 
	UDR = string;
	while ((UCSRA & (1 << UDRE)) == 0) {};
	UDR = 0x0D; //CR
	while ((UCSRA & (1 << UDRE)) == 0) {};
	UDR = 0x0A; //LF
}

//help overview, printed everytime when going to main menu
void println_P(uint8_t min, uint8_t max)
{
	char lineBuffer[64];
	uint8_t i;
	for(i = min; i <= max; i++)
	{
		strcpy_P(lineBuffer,helpLine[i]);
		println(lineBuffer);
	}
	
	//println("6) print motor map");
}

//set pins to output after startup
void prepare_pins (void)
{
	MCUCSR = (1<<JTD);
    MCUCSR = (1<<JTD);
	//set ports A, B and C to output
	DDRA = 0xFF;
	DDRB = 0xFF;
	DDRC = 0xFF;
	
	//set port D 2 to 7 to output, 0 and 1 are for RS-232
    DDRD = (1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7);

	//prepare RS-232
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1<<RXCIE);
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes

	UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
	UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
}

//prepare timer1 for interrupt usage
void prepare_timer()
{
	//timer1 needs to interrupt on a regular basis -> every 10 ms
	
	//set prescaler to 64 cycles, set clear timer1/counter on compare match
	TCCR1B |= (1 << WGM12)|(1 << CS12); //256 cycles count
	//->62500 cycle counts per second
	//->63 cycle counts per millisecond
	//->625 cycle counts per 10 ms
	
    // initialize counter
    TCNT1 = 0;
	
	//set Timer/Counter Output Compare Register Timer/Counter 1 to 2000 cycles (=1ms) or 20000 (=10ms)
	OCR1A = 625;
	
	//enable timer compare interrupt
	TIMSK |= (1 << OCIE1A);
	

}

//set pins min to max of port pport to high for 250ms
void test_pins(volatile uint8_t *pport, uint8_t min, uint8_t max)
{
	char buffer[3];
	uint8_t i;
	for(i = min; i <= max; i++)
	{
		*pport = 0x00 | (1 << i);
		println(itoa(i, buffer, 10));
		sleep(250);
		sleep(250);
		
	}
	*pport = 0x00;
}

void test_motors()
{
	int i;
	for(i = 0; i < 30; i++)
	{
		switchMotorOn(i);
		sleep(250);
		sleep(250);
		switchMotorOff(i);
	}
}

//go through all ports and test pins for them
void test_loop (void)
{
	println_P(18,18);
	test_pins(&PORTA, 0, 7);
	println_P(19,19);
	test_pins(&PORTB, 0, 7);
	println_P(20,20);
	test_pins(&PORTC, 0, 7);
	println_P(21,21);
	test_pins(&PORTD, 2, 7);
}


void switchMotorOn(uint8_t pinID)
{
	uint8_t portCounter = 0;
	for(;;)
	{
		if(pinID < 8)
		{
			if(portCounter == 0) //PORTA
			{
				PORTA |= ( 1 << pinID );
				return;
			}
			if(portCounter == 1) //PORTB
			{
				PORTB |= ( 1 << pinID );
				return;
			}
			if(portCounter == 2) //PORTC
			{
				PORTC |= ( 1 << pinID );
				return;
			}
			if(portCounter == 3) //PORTD
			{
				PORTD |= ( 1 << (pinID + 2) );
				return;
			}
			if(portCounter >=4)
			{
					return;
			}
		}
		pinID = pinID - 8;
		portCounter++;
	}
}

void switchMotorOff(uint8_t pinID)
{
	uint8_t portCounter = 0;
	for(;;)
	{
		if(pinID < 8)
		{
			if(portCounter == 0) //PORTA
			{
				PORTA &= ( 0 << pinID );
				return;
			}
			if(portCounter == 1) //PORTB
			{
				PORTB &= ( 0 << pinID );
				return;
			}
			if(portCounter == 2) //PORTC
			{
				PORTC &= ( 0 << pinID );
				return;
			}
			if(portCounter == 3) //PORTD
			{
				PORTD &= ( 0 << (pinID + 2) );
				return;
			}
			if(portCounter >=4)
			{
					return;
			}
		}
		pinID = pinID - 8;
		portCounter++;
	}
}

void decreaseMotorTimers(void)
{
	uint8_t n;
	for(n = 0; n <= 29; n++)
	{
		if(driveIRCountsToShutdown[n] > 0)
		{
			driveIRCountsToShutdown[n]--;
			if(driveIRCountsToShutdown[n] == 0)
			{
				switchMotorOff(n);
			}
		}
	}
}

void initializeEEPROM(void)
{
	
	println_P(22,22);
	int n;
	
	for(n = 0; n < 128; n++)
	{
		driveMotors[n][0] = -1;
		driveMotors[n][1] = -1;
		driveMotors[n][2] = -1;
	}
	
	driveMotors['a'][0] = 26;
	driveMotors['a'][1] = 25;
	driveMotors['b'][0] = 28;
	driveMotors['b'][1] = 10;
	driveMotors['c'][0] = 14;
	driveMotors['c'][1] = 15;
	driveMotors['d'][0] = 13;
	driveMotors['d'][1] = 29;
	driveMotors['e'][0] = 22;
	driveMotors['e'][1] = 7;
	driveMotors['f'][0] = 23;
	driveMotors['f'][1] = 5;
	driveMotors['g'][0] = 4;
	driveMotors['g'][1] = 6;
	driveMotors['h'][0] = 21;
	driveMotors['h'][1] = 19;
	driveMotors['i'][0] = 18;
	driveMotors['i'][1] = 20;
	driveMotors['j'][0] = 3;
	driveMotors['j'][1] = 2;
	driveMotors['k'][0] = 0;
	driveMotors['k'][1] = 16;
	driveMotors['l'][0] = 1;
	driveMotors['l'][1] = 17;
	driveMotors['m'][0] = 8;
	driveMotors['m'][1] = 9;
	driveMotors['n'][0] = 12;
	driveMotors['n'][1] = 11;
	driveMotors['o'][0] = 15;
	driveMotors['o'][1] = 16;
	driveMotors['p'][0] = 29;
	driveMotors['p'][1] = 2;
	driveMotors['q'][0] = 20;
	driveMotors['q'][1] = 17;
	driveMotors['r'][0] = 14;
	driveMotors['r'][1] = 0;
	driveMotors['s'][0] = 13;
	driveMotors['s'][1] = 3;
	driveMotors['t'][0] = 18;
	driveMotors['t'][1] = 1;
	driveMotors['u'][0] = 25;
	driveMotors['u'][1] = 6;
	driveMotors['v'][0] = 10;
	driveMotors['v'][1] = 5;
	driveMotors['w'][0] = 7;
	driveMotors['w'][1] = 19;
	driveMotors['x'][0] = 26;
	driveMotors['x'][1] = 4;
	driveMotors['y'][0] = 28;
	driveMotors['y'][1] = 23;
	driveMotors['z'][0] = 22;
	driveMotors['z'][1] = 21;


	driveMotors['0'][0] = 20;
	driveMotors['1'][0] = 21;
	driveMotors['2'][0] = 22;
	driveMotors['3'][0] = 23;
	driveMotors['4'][0] = 24;
	driveMotors['5'][0] = 25;
	driveMotors['6'][0] = 26;
	driveMotors['7'][0] = 27;
	driveMotors['8'][0] = 28;
	driveMotors['9'][0] = 29;
	
	/*
	driveMotors['a'][0] = 0;
	driveMotors['a'][1] = 10;
	driveMotors['b'][0] = 1;
	driveMotors['b'][1] = 11;
	driveMotors['c'][0] = 2;
	driveMotors['c'][1] = 12;
	driveMotors['d'][0] = 3;
	driveMotors['d'][1] = 15;
	driveMotors['e'][0] = 4;
	driveMotors['e'][1] = 13;
	driveMotors['f'][0] = 5;
	driveMotors['f'][1] = 0;
	driveMotors['g'][0] = 6;
	driveMotors['g'][1] = 16;
	driveMotors['h'][0] = 7;
	driveMotors['h'][1] = 25;
	driveMotors['i'][0] = 8;
	driveMotors['i'][1] = 28;
	driveMotors['j'][0] = 9;
	driveMotors['j'][1] = 19;
	driveMotors['k'][0] = 10;
	driveMotors['k'][1] = 20;
	driveMotors['l'][0] = 11;
	driveMotors['l'][1] = 21;
	driveMotors['m'][0] = 12;
	driveMotors['m'][1] = 22;
	driveMotors['n'][0] = 13;
	driveMotors['n'][1] = 23;
	driveMotors['o'][0] = 14;
	driveMotors['o'][1] = 24;
	driveMotors['p'][0] = 15;
	driveMotors['p'][1] = 25;
	driveMotors['q'][0] = 16;
	driveMotors['q'][1] = 26;
	driveMotors['r'][0] = 17;
	driveMotors['r'][1] = 27;
	driveMotors['s'][0] = 18;
	driveMotors['s'][1] = 28;
	driveMotors['t'][0] = 19;
	driveMotors['t'][1] = 29;
	driveMotors['u'][0] = 20;
	driveMotors['u'][1] = 0;
	driveMotors['v'][0] = 21;
	driveMotors['v'][1] = 1;
	driveMotors['w'][0] = 22;
	driveMotors['w'][1] = 2;
	driveMotors['x'][0] = 23;
	driveMotors['x'][1] = 3;
	driveMotors['y'][0] = 24;
	driveMotors['y'][1] = 4;
	driveMotors['z'][0] = 25;
	driveMotors['z'][1] = 5;
	*/
	/*//small alphabet
	for(n = 97; n <= 122; n++)
	{
		driveMotors[n][0] = n - 97; //motors 0 to 25
	}

	//small alphabet second motor first half alphabet
	for(n = 97; n <= 116; n++)
	{
		driveMotors[n][1] = n - 87; //motors 0 to 25
	}
	//small alphabet second motor second half alphabet
	for(n = 117; n <= 122; n++)
	{
		driveMotors[n][1] = n - 117; //motors 0 to 25
	}

	//capital alphabet
	for(n = 65; n <= 90; n++)
	{
		driveMotors[n][0] = 26;
		driveMotors[n][1] = n - 65; //motors 0 to 25
	}
	
	//numbers
	for(n = 48; n <= 57; n++)
	{
		driveMotors[n][0] = 27;
		driveMotors[n][1] = n - 48;
	}
	*/
	eeprom_write_block (driveMotors, driveMotorsEEPROM, sizeof(driveMotors));
	
	println_P(23,23);

}

void showMotorMap()
{
	println_P(16,17);
	uint8_t n;
	for(n = 1; n < 128; n++)
	{
		char myBuffer[4] = {' ',' ',' ','\0'};
		char myOutput[16] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ','\0','\0'};
		myOutput[0] = (char) n;
		myOutput[1] = ':';
		itoa(n, myBuffer, 10);
		myOutput[2] = myBuffer[0];
		if(myBuffer[1] != '\0')
			myOutput[3] = myBuffer[1];
		if(myBuffer[2] != '\0')
			myOutput[4] = myBuffer[2];
		myOutput[5] = ':';
		itoa(driveMotors[n][0], myBuffer, 10);
		myOutput[6] = myBuffer[0];
		if(myBuffer[1] != '\0')
			myOutput[7] = myBuffer[1];
		myOutput[8] = ',';
		itoa(driveMotors[n][1], myBuffer, 10);
		myOutput[9] = myBuffer[0];
		if(myBuffer[1] != '\0')
			myOutput[10] = myBuffer[1];
		myOutput[11] = ',';
		itoa(driveMotors[n][2], myBuffer, 10);
		myOutput[12] = myBuffer[0];
		if(myBuffer[1] != '\0')
			myOutput[13] = myBuffer[1];
		println(myOutput);
	}
	println_P(17,17);
}

void changeMotorMap(void)
{

	println_P(14,15);
	
	char ReceivedByte = '\0';
	
	for(;;)
	{
		while ((UCSRA & (1 << RXC)) == 0) {};
		ReceivedByte = UDR;
		uint8_t n;
		println_P(13,13);
		for(n = 0; n < 3; n++)
		{
			driveMotors[(int)ReceivedByte][n] = getNumber();
		}
	}
}

uint8_t getNumber()
{
	char myBuffer[4] = {' ',' ',' ','\0'};
	uint8_t bufferPos = 0;
	uint8_t retVal = 0;
	
	for(;;)
	{
		while ((UCSRA & (1 << RXC)) == 0) {};
		myBuffer[bufferPos] = UDR;
		if(myBuffer[bufferPos] == ';')
		{
			retVal = atoi(myBuffer);
			return retVal;
		}
		else
		{
			bufferPos++;
		}
	}
}

int main (void)
{
	char ReceivedByte;

	prepare_pins();
	
	prepare_timer();
	
	eeprom_read_block (driveMotors, driveMotorsEEPROM, sizeof(driveMotors));

	println_P(3,3);
	println_P(0,11); //help

	for (;;) // Loop forever
	{
		while ((UCSRA & (1 << RXC)) == 0) {}; // Do nothing until data have been received and is ready to be read from UDR
		ReceivedByte = UDR; // Fetch the received byte value into the variable "ByteReceived"
		//printchr(ReceivedByte);
		if(ReceivedByte == '1') //start input
		{
			//println();
			//enable interrupts
			sei();
			
			println_P(24,24);
			
			
			//BEGIN INPUT LOOP
			for(;;)
			{
				//on timer interrupt 10 ms passed. decrease motor timings and stop motors with timeout
				if(0x01 == (programState &= 0x01))
				{
					programState ^= 0x01;
					//println("timer interrupt recognized");
					decreaseMotorTimers();
				}
				
				//get character from ring buffer
				//disable interrupt not required because only atomic compare operation matters. it is assumed that the buffer can not overflow
				//UCSRB ^= (1<<RXCIE); //disable UART interrupt
				if(bufferUART.read != bufferUART.write)
				{
					ReceivedByte = bufferUART.ring[bufferUART.read];
					if(bufferUART.read < RING_BUFFER_SIZE-1)
						bufferUART.read++; // overflow??
					else
						bufferUART.read = 0;
					//println("received uart byte");
					//printchr(ReceivedByte);
					
					if(ReceivedByte == '_')
						break;

					int x;
					for(x = 0; x < 3; x++)
					if(driveMotors[(int)ReceivedByte][x] != -1)
					{
						//println("drive");
						//printchr(ReceivedByte);
						driveIRCountsToShutdown[driveMotors[(int)ReceivedByte][x]] = 20; //enable motor for x*10 ms (=300ms)
						switchMotorOn(driveMotors[(int)ReceivedByte][x]);
					}
				}
				//UCSRB ^= (1<<RXCIE); //enable UART interrupt
			}
			//END INPUT LOOP
			
			//disable interrupts
			cli();
			
			println_P(0,11); //help
			
		}
		if(ReceivedByte == '2') //change character assignment
		{
			showMotorMap();
			
			changeMotorMap();
			
			eeprom_write_block (driveMotors, driveMotorsEEPROM, sizeof(driveMotors));
			
			println_P(0,11); //help
		}
		if(ReceivedByte == '3') //change motor timings
		{
			
		}
		if(ReceivedByte == '4') //testloop
		{
			println_P(12,12); //help
			test_loop();
			println_P(0,11); //help
		}
		if(ReceivedByte == '5') //initialize eeprom
		{
			initializeEEPROM();
			println_P(0,11); //help
		}
		if(ReceivedByte == '6')
		{
			showMotorMap();
		}
		if(ReceivedByte == '7')
		{
			test_motors();
		}
	}
}

ISR(TIMER1_COMPA_vect) //timer interrupt
{
	programState |= 0x01; //bit1: decrease motor time
}

ISR(USART_RXC_vect) //UART interrupt
{
	bufferUART.ring[bufferUART.write] = UDR;
	if(bufferUART.write < (RING_BUFFER_SIZE - 1))
		bufferUART.write++;
	else
		bufferUART.write = 0;
}
