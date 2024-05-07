/*
 * led-controller.cpp
 *
 * Created: 16/04/2024 7:16:16 PM
 * Author : jmcna
 */ 

// Clock params
#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

// USART 2 params
#define USART_BAUD_RATE(BAUD_RATE) ((float)(8000000UL * 64 / (16 * (float)BAUD_RATE)) + 0.5)
#define MAX_INPUT_LEN 512
#define MAX_NUM_COMMANDS 128
#define MAX_NUM_INTS 3*MAX_NUM_COMMANDS

// Analog out params
#define DAC_BLUE 1
#define DAC_WHITE 0

// Timer params
#define TCB0_TOP 0x0190
#define TCB1_TOP 0x4E20
#define TCB2_TOP 0x0FA0

// LED params
#define MAX_AVG_CURRENT  2067 // mA
#define MAX_V_SET        2400 // mV
#define CURRENT_SLEW     200   // mA/us
inline uint8_t getShortTime(const uint16_t& vset) {return vset * MAX_AVG_CURRENT / MAX_V_SET / CURRENT_SLEW;}

// Globals
uint16_t dac_white_value = 0;
uint16_t dac_blue_value = 0;
volatile uint32_t time = 0; // (us)
uint16_t white_led_on_time = 0; // (ms)
uint16_t white_led_brightness = 320; // DAC value

// These are the specific actions that
enum Action
{
	WRITE_DAC_WHITE,
	WRITE_DAC_BLUE,
	BLUE_PULSE,
	WHITE_PULSE,
	SET_DAC_WHITE,
	TRIGGER_PULSE
};

struct Command
{
	int action;
	uint32_t time;
	uint32_t value;
};

void CLOCK_init()
{
	// Set internal oscillator at 8Mhz with autotune enabled
	_PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, CLKCTRL_FRQSEL_8M_gc);

	// Enable 32KHz RTC to work with autotune
	//_PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA,  CLKCTRL_ENABLE_bm);

	// Don't enable the clock prescale
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, ~(CLKCTRL_PEN_bm));

	// Select internal HF oscillator
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCHF_gc);
}

void USART2_sendChar(char c)
{
	// Wait until transmit data register is empty
	while (!(USART2.STATUS & USART_DREIF_bm))
	{
		;
	}
	
	// Send data
	USART2.TXDATAL = c;
}

// Initialize USART2 in their default pin locations
void USART2_init(uint16_t baud)
{
	PORTF.DIR |= PIN0_bm;
	PORTF.DIR &= ~PIN1_bm;
	
	// Set the baud rate
	USART2.BAUD = uint16_t(USART_BAUD_RATE(baud));
	
	// Enable transmitter and receiver
	USART2.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;
	
	// Set asynchronous, no parity, 8bit data, 1 stop bit
	// This is the default configuration but we set it explicitly for clarity
	USART2.CTRLC |= USART_PMODE0_bm | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;

	// Initial serial garbage makes messages fail if they occur too soon after init
	_delay_ms(32);
}

void SPI0_init()
{
	// Set MOSI, SCK, and SS to output
	PORTA.DIR |= PIN4_bm | PIN6_bm | PIN7_bm;
	
	// Set endianness, master mode, /4 prescale, no clock doubling, SPI enable
	SPI0.CTRLA = ~(SPI_DORD_bm) | SPI_MASTER_bm & (~SPI_CLK2X_bm) | SPI_PRESC_DIV4_gc | SPI_ENABLE_bm;
}

uint8_t SPI0_write(uint8_t data)
{
	SPI0.DATA = data;
	
	while (!(SPI0.INTFLAGS & SPI_IF_bm))
	{
		;
	}
	return SPI0.DATA;
}

void writeDACn(uint16_t n, uint16_t value)
{
	value = value > 4096 ? 4096 : value;

	// Add configuration bits for DAC selection (a/b) and normal operation
	value |= (n >= 0x01) << 15 & ~(0x01 << 13) | 0x01 << 12;

	// Send SPI
	PORTA.OUT &= ~PIN7_bm;
	SPI0_write(value >> 8);
	SPI0_write(value);
	PORTA.OUT |= PIN7_bm;
}

void GPIO_init(void)
{
	// Set the direction of the GPIO pins we use
	PORTD_DIR |= PIN0_bm;	// PWM BLUE
	PORTD_DIR |= PIN1_bm;	// PWM WHITE
	PORTD_DIR |= PIN2_bm;	// SHORT BLUE
	PORTC_DIR |= PIN2_bm;	// TRIGGER
	
	// Set all digital outputs to LOW
	PORTD_OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm;
	PORTC_OUTCLR = PIN2_bm;
	
	writeDACn(DAC_BLUE, 0);
	writeDACn(DAC_WHITE, 0);
}

uint8_t USART2_readChar()
{
	// Wait for unread data in the receive buffer
	while(!(USART2.STATUS & USART_RXCIF_bm))
	{
		;
	}
	
	return USART2.RXDATAL;
}

void USART2_sendString(char *str)
{
	for(size_t i = 0; i < strlen(str); i++)
	{
		USART2_sendChar(str[i]);
	}
}

void USART2_readString(char* str)
{
	USART2_sendString("Started reading\n");
	char c;
	uint8_t index = 0;

	while (1)
	{
		c = USART2_readChar();
		if(c != '\n' && c != '\r')
		{
			str[index++] = c;
		
			// overwrite old data if it overruns the input buffer
			if(index > MAX_INPUT_LEN)
			{
				index = 0;
			}
		}

		if(c == '\n')
		{
			str[index] = '\0';
			index = 0;
			USART2_sendString("char!\n");
			return;
		}
	}
}

bool parse(const char* c, char** end, Command* cmds, size_t& idx)
{
	//printf("Parsing '%s':\n", c);
	uint32_t buffer[MAX_NUM_INTS];
	uint8_t n = 0;
	*end = (char*)c;

	USART2_sendString("Started parsing\n");
	while (n < MAX_NUM_INTS)
	{
		// Reset errno, as its used by many std functions
		errno = 0;

		// Attempt to extract a long from the string
		const uint32_t val = strtoul(c, end, 10);

		// Check if there was a range error
		const bool range_error = errno == ERANGE;

		// Conversion failed, stop
		if (c == *end || range_error)
			break;

		// Update the start of the string to where the last number extracted finished
		c = *end;

		// Add the new number to the buffer
		buffer[n] = val;

		// Increment counter
		n++;
	}

	// Need multiple of 3 to make valid commands
	if (n % 3 != 0 || n == 0)
	{
		return false;
	}

	idx = n / 3;
	for(int i = 0; i < n; i += 3)
	{
		cmds[i/3] = {int(buffer[i]), buffer[i+1], buffer[i+2] };
	}

	return true;
}

void TCB0_init(void)
{
	// Default mode and clock source are OK
	
	// Enable overflow interrupt
	TCB0.INTCTRL = TCB_CAPT_bm;
	
	// Set TOP value to correspond with 50us pulse width
	TCB0.CCMP = TCB0_TOP;
}

void TCB1_init(void)
{
	// Default mode and clock source are OK
	
	// Enable overflow interrupt
	TCB1.INTCTRL = TCB_CAPT_bm;
	
	TCB1.CTRLA |= TCB_CLKSEL_DIV2_gc;
	
	// Set TOP value to correspond with 50us pulse width
	TCB1.CCMP = TCB1_TOP;
}

void TCB2_init(void)
{
	// Default mode and clock source are OK
	
	// Enable compare interrupt
	TCB2.INTCTRL = TCB_CAPT_bm;
	
	TCB2.CTRLA |= TCB_CLKSEL_DIV2_gc;
	
	// Set TOP value for 10us time
	TCB2.CCMP = TCB2_TOP;
	
	// Start
	TCB2.CTRLA |= TCB_ENABLE_bm;
}

void TCA0_init(void)
{
	/* set Normal mode */
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
	
	// Enable count interrupts for both count 0, 1
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm;
	/* disable event counting */
	TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTAEI_bm);
	// Set prescale
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
}

ISR(TCA0_CMP0_vect)
{
	PORTD_OUTCLR = PIN2_bm;
	TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP0_bm;
}

ISR(TCA0_CMP1_vect)
{
	PORTD_OUTCLR = PIN0_bm;
	TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm;
	
	TCA0.SINGLE.INTCTRL &= ~TCA_SINGLE_CMP0EN_bm & ~TCA_SINGLE_CMP1EN_bm;
	
	TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
}

ISR(TCB0_INT_vect)
{
	// Set the trigger low
	PORTC_OUTCLR = PIN2_bm;
	
	// Turn off interrupt
	TCB0.INTFLAGS |= TCB_CAPT_bm;
	TCB0.CTRLA &= ~TCB_ENABLE_bm;
}

ISR(TCB1_INT_vect)
{
	static uint16_t counter = 0;
	
	// Increment time, we have 8Mhz clock with DIV2 prescale so
	// milliseconds = count / 4000
	counter += TCB1_TOP / 4000;
	
	if (counter >= white_led_on_time)
	{
		// Turn off interrupt
		TCB1.INTCTRL = 0;
		
		// Turn off white LED
		writeDACn(DAC_WHITE, 0);
		
		// Reset counter for next time
		counter = 0;
		
		// Disable interrupt
		TCB1.INTCTRL &= ~TCB_CAPT_bm;
	}
	TCB1.INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB2_INT_vect)
{
	// +1 ms
	++time;
	
	// Clear flag
	TCB2.INTFLAGS |= TCB_CAPT_bm;
}

void delayMicroseconds(uint16_t us)
{
	// for the 8 MHz internal clock

	// for a 1 and 2 microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 2us
	if (us <= 2) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/2 of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1; //x2 us, = 2 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 17 (19) cycles above, remove 4, (4*4=16)
	// us is at least 6 so we can substract 4
	us -= 4; // = 2 cycles
	
	
	// busy wait
	__asm__ __volatile__ (
	"1: sbiw %0,1" "\n\t" // 2 cycles
	"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
	// return = 4 cycles
}

void pulse(uint8_t tshort, uint32_t ton)
{
	// Reduce tshort by 2us, the overhead in enabling the timer
	if (tshort > 2)
	tshort -= 2;
	else
	tshort = 0;
	
	// Ton could range from 10us to 1000000us in theory. If it is larger than 8191us, change the prescale
	if (ton > (uint16_t(~0) >> 3))
	{
		USART2_sendString("Pulse Big\n");
		// Set prescale to x256 T = 32 (us)
		TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV256_gc;
		
		// Enable interrupts
		TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1EN_bm;
		
		// Set compare value
		TCA0.SINGLE.CMP1 = (tshort + ton) >> 5;
		
		// Enable
		TCA0.SINGLE.CNT = 0;
		TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
		
		// Set pins ON
		PORTD_OUTSET = PIN0_bm | PIN2_bm;
		
		// Wait tshort microseconds and turn OFF short
		delayMicroseconds(tshort+2);
		PORTD_OUTCLR = PIN2_bm;
	}
	else
	{
		USART2_sendString("Pulse\n");
		// Enable interrupts
		TCA0.SINGLE.INTCTRL |= TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm;
		
		// Set prescale DIV1
		TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
		
		// Set compare values
		TCA0.SINGLE.CMP0 = tshort << 3;
		TCA0.SINGLE.CMP1 = (tshort + ton) << 3;
		
		// Enable
		TCA0.SINGLE.CNT = 0;
		TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
		
		// Set pins ON
		PORTD_OUTSET = PIN0_bm | PIN2_bm;
	}
}

void execute(Command* cmds, size_t idx)
{
	time = 0;

	uint16_t i = 0;
	while(i < idx)
	{		
		if (cmds[i].time <= time)
		{
			switch (cmds[i].action)
			{
				case WRITE_DAC_WHITE:
					dac_white_value = cmds[i]. value;
					writeDACn(DAC_WHITE, dac_white_value);
					break;
				case WRITE_DAC_BLUE:
					dac_blue_value = cmds[i].value;
					writeDACn(DAC_BLUE, dac_blue_value);
					break;
				case BLUE_PULSE:
					pulse(getShortTime(dac_blue_value), cmds[i].value);
					break;
				case WHITE_PULSE:
					TCB1.CNT = 0x00;
					// Turn on interrupt
					TCB1.INTCTRL = TCB_CAPT_bm;
					
					// Set on time
					white_led_on_time = cmds[i].value;
										
					// Turn on white LED
					writeDACn(DAC_WHITE, white_led_brightness);
					
					// Start timer B
					TCB1.CTRLA |= TCB_ENABLE_bm;
					
					break;
				case SET_DAC_WHITE:
					white_led_brightness = cmds[i].value;
					break;
					
				case TRIGGER_PULSE:
					// Reset timer count and top value
					TCB0_CNT = 0x00;
					
					// Turn on interrupt
					TCB0.INTCTRL = TCB_CAPT_bm;
					
					// Start timer B
					TCB0.CTRLA = TCB_ENABLE_bm;
					
					// Write pin high
					PORTC_OUTSET = PIN2_bm;
					break;
				default:
					break;
			}
			i++;	
		}
	}
}

int main(void)
{
	// Initialise modules
	CLOCK_init();
	SPI0_init();
	GPIO_init();
	TCA0_init();
	TCB0_init();
	TCB1_init();
	TCB2_init();
	USART2_init(9600);
	sei();
			
	// Receive buffer for all commands
	char buffer[MAX_INPUT_LEN];
	Command commands[MAX_NUM_COMMANDS];
	
	while (1)
	{
		// Wait for a line
		USART2_readString(buffer);
		
		char * end;
		size_t idx = 0;
		if ( parse(buffer, &end, commands, idx) )
		{
			USART2_sendString("Good parse!\n");
			for(int i = 0; i < idx; i++)
			{
				sprintf(buffer, "{\n\t%d \n\t%lu \n\t%lu \n}\n", commands[i].action, commands[i].time, commands[i].value);
				USART2_sendString(buffer);
			}
			execute(commands, idx);
		}
		else
		{
			USART2_sendString("Bad parse!\n");
			USART2_sendString(end);
		}
		
	}
}

