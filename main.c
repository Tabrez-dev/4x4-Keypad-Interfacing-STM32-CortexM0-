#include<stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>



int main(void);
void reset_handler(void);
void system_clock_config(void);

extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;

extern uint32_t _sbss;
extern uint32_t _ebss;

#define SRAM_START 0x20000000U
#define SRAM_SIZE (16 * 1024) //16K
#define SRAM_END ((SRAM_START) + (SRAM_SIZE))

#define STACK_START SRAM_END

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank)-'A')<<8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
//#define SYSTICK_ENABLE
struct gpio {
	volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFRL, AFRH, BRR; 
};
#define GPIO(bank) ((struct gpio *) (0x48000000 + 0x400 * (bank)))
//Enum vlaues are per datasheet: 0,1,2,3...for specifc function
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

//In order to enable a GPIO peripheral, it should be enabled (clocked) via the RCC (Reset and Clock Control) unit.

struct rcc {
	volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR, APB1ENR, BDCR, CSR, AHBRSTR, CFGR2, CFGR3, CR2;
};
#define RCC ((struct rcc *) 0x40021000)
#define RCC_GPIO_CLK_ENABLE(bank)  (RCC->AHBENR |= BIT(17 + (bank)))
#define RCC_GPIO_CLK_DISABLE(bank) (RCC->AHBENR &= ~BIT(17 + (bank)))

#define SYSTICK ((struct systick *) 0xE000E010)
#define UART1 ((struct uart *)0x40013800)
#define FREQ 8000000  // HSI clock frequency is 8 MHz by default

uint32_t udiv32(uint32_t dividend, uint32_t divisor) {
	if (divisor == 0) {
		// Handle division by zero (return an error value or throw an exception if you have one)
		return UINT32_MAX; // Or another appropriate error value
	}

	uint32_t quotient = 0;
	uint32_t remainder = dividend;

	for (int i = 31; i >= 0; i--) {
		if (remainder >= divisor) {
			quotient |= (1UL << i);
			remainder -= (divisor << i);
		}
	}
	return quotient;
}
static inline void spin(volatile uint32_t count) {
	while (count--) (void) 0;
}


void system_clock_config(void){

	uint32_t volatile *const pRccCR = (uint32_t*)(0x40021000);
	uint32_t volatile *const pRccCfgr = (uint32_t*)(0x40021004);

	*pRccCR |= (1<<0);
	*pRccCfgr &= ~(3<<0);
	*pRccCfgr &= (1<<0); 

}

void reset_handler(void){

	//copy initialised data in .data to RAm from ROM
	uint32_t size= &_edata - &_sdata;

	uint8_t *pDst = (uint8_t*)&_sdata;//sram
	uint8_t *pSrc = (uint8_t*)&_etext;//flash

	for(uint32_t i=0;i<size;i++){
		*pDst++=*pSrc++;

	}

	//init the .bss section to 0 in sram
	size = &_ebss - &_sbss;
	pDst = (uint8_t*)&_sbss;
	for(uint32_t i=0;i<size;i++){
		*pDst++=0;
	}

	system_clock_config();

	main();

}

__attribute__((section(".isr_vector")))
const uint32_t vector_table[]=
{
	STACK_START,//MSP at end of SRAM 128K
	(uint32_t)&reset_handler,//function pointer
	[2 ... 38]=0


};

void delay(void){
	//for 8Mhz clock-> 1 ins->0.125 micro sec,  1ms->8000 ins, 200ms-> 1600000 ins
	for(volatile uint32_t i=0;i< 1600000;i++);//check in dissassembly how many ins does delay take
}


// -------------------------- GPIO Functionality --------------------------
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
	// Get the GPIO bank (port) for the specified pin (A, B, C, etc.)
	struct gpio *gpio = GPIO(PINBANK(pin));
	// Extract the pin number (0-15) from the pin identifier (e.g., 'A5' -> 5)
	int n = PINNO(pin);

	RCC_GPIO_CLK_ENABLE(PINBANK(pin));//verify if this works
					  // Clear the existing 2-bit mode configuration for the specific pin
					  // The 2 bits for each pin are at positions 2*n and 2*n+1 in MODER.
	gpio->MODER &= ~(3U << (n * 2));
	// Set the new mode by OR'ing the correct mode value at the proper position
	// (mode & 3) ensures we only take the lower 2 bits of the mode
	// and then shift them to the correct position for the pin.
	gpio->MODER |= (mode & 3U) << (n * 2);
}

// Define a function to write a value (high/low) to a specific GPIO pin
static inline void gpio_write(uint16_t pin, bool val) {
	// Access the GPIO structure corresponding to the pin's bank (group of pins)
	struct gpio *gpio = GPIO(PINBANK(pin));
	// Write the value to the pin's bit set/reset register (BSRR).
	// The value is shifted left based on whether 'val' is true or false.
	// If 'val' is true (high), it sets the corresponding pin high.
	// If 'val' is false (low), it resets the pin (sets it low).
	gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}



// -------------------------- UART Functionality --------------------------

struct uart {
	volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR;
}; 

static inline void gpioSetAF(uint16_t pin, uint8_t afNum){

	struct gpio *gpio = GPIO(PINBANK(pin));
	int n = PINNO(pin);

	if(n<8){
		gpio->AFRL &= ~(0xFUL << (n * 4)); //clear AF bit sin AFRL
		gpio->AFRL |= ((uint32_t)afNum << (n *4));
	} else {
		gpio->AFRH &= ~(0xFUL << ((n-8) * 4));
		gpio->AFRH |= ((uint32_t)afNum << ((n-8) * 4));
	}
}

//https://www.st.com/resource/en/datasheet/stm32f072rb.pdf
//Only enables USART1

static inline void uartInit(struct uart *uart , unsigned long baud){
	uint8_t af=1;		//AF1 for USART1 from AF memory mapping
	uint16_t rx=0,tx=0;	

	//Enable Clock for the selected UART
	if(uart == UART1) RCC->APB2ENR |= BIT(14);

	//configure the tx and rx pins
	if(uart == UART1){
		tx = PIN('A',9);
		rx = PIN('A',10);
	}
	//Configure GPIO for UART pins in AF mode
	gpio_set_mode(tx, GPIO_MODE_AF);
	gpioSetAF(tx,af);
	gpio_set_mode(rx, GPIO_MODE_AF);
	gpioSetAF(rx,af);

	//Configure the UART
	uart->CR1 = 0; 			//Disable the UART before configuring
					// uart->BRR = FREQ / baud; 		//Set Baud rate (FREQ is the UART clock frequency) 
	uart->BRR = udiv32(FREQ, baud); 
	uart->CR1 |= BIT(0) | BIT(2) | BIT(3);//Enable UART, RX, TX
}

//Check if UART RX data is ready
static inline int uartReadReady(struct uart *uart){
	return uart->ISR & BIT(5); //Check RXNE(Receiver Not Empty) (bit5 in ISR register)

}

//Read a single Byte from UART
static inline uint8_t uartReadByte(struct uart *uart){
	return (uint8_t)(uart->RDR & 0xFF); //Read 8 bit data from RDR reg
}

//write a single byte to UART 
static inline void uartWriteByte(struct uart *uart, uint8_t byte){
	while((uart->ISR & BIT(7)) == 0) spin(1); //Wait untill TXE (bit 7 in ISR register) is set
	uart->TDR = byte; //write byte to transmit data register

}

//write a buffer to UART
static inline void uartWriteBuf(struct uart *uart, char *buf, size_t len){
	while(len-- > 0) {
		uartWriteByte(uart, *(uint8_t *) buf++);
	}

}



int main(){
	//memory mapped i/o registers are volatile const pointers because their addresss should not be changed and data should not be optimised
	uint32_t volatile *const pGPIOBModeReg = (uint32_t*)(0x48000400);
	uint32_t volatile *const pInPutDataReg = (uint32_t*)(0x48000400+0x10);
	uint32_t volatile *const pOutPutDataReg = (uint32_t*)(0x48000400+0x14);
	uint32_t volatile *const pClockCtrlReg = (uint32_t*)(0x40021000+0x14);//RCC_AHBENR
	uint32_t volatile *const pPullupDownReg = (uint32_t*)(0x48000400+0x0C);
	//1. Enable the peripheral clock of GPIOB peripheral
	*pClockCtrlReg |= (1 << 18);
	//2. Configure PB2,PB3,PB4 and PB5 as output (rows)
	*pGPIOBModeReg &= ~(0xFF << 4); //clear
	*pGPIOBModeReg |= (0x55 << 4); //set
				       //3. Configure PB6, PB7, PB8 and PB9 as input (columns)
	*pGPIOBModeReg &= ~(0xFF << 12); 
	//4. Enable internal pull up registers for PB6,PB7,PB8 and PB9 
	*pPullupDownReg &= ~(0xFF << 12);
	*pPullupDownReg |=  (0x55 << 12);

	uartInit(UART1, 115200);
	//    char message[] = "hi\r\n";

	char message[20];
	// Initial message:
	message[0] = 'U';
	message[1] = 'A';
	message[2] = 'R';
	message[3] = 'T';
	message[4] = ' ';
	message[5] = 'T';
	message[6] = 'X';
	message[7] = '\r';
	message[8] = '\n';
	message[9] = '\0'; 
	uartWriteBuf(UART1, message, 10);

	while(1){
		//Make all row GPIOs High state
		*pOutPutDataReg |= (0x0F << 2);
		//Make R1 Low state R1=0,R2=1,R3=1,R4=1(PB2)
		*pOutPutDataReg &= ~(1 << 2);

		//Read C1, is C1 low?(PB6)
		//if PB6 is high then false else true so we use not operator
		if( !(*pInPutDataReg & (1 << 6)) ){
			//key pressed
			//debouncing logic
			delay();
			//printf("1\n");
			// Rewrite the message:
			message[0] = '1';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0'; // Important: Null-terminate again

			uartWriteBuf(UART1, message, 4);
		}
		//C2
		if( !(*pInPutDataReg & (1 << 7)) ){
			//key pressed
			delay();
			//printf("2\n");
			// Construct the message:
			message[0] = '2';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C3
		if( !(*pInPutDataReg & (1 << 8)) ){
			//key pressed
			delay();
			//printf("3\n");
			// Construct the message:
			message[0] = '3';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C4
		if( !(*pInPutDataReg & (1 << 9)) ){
			//key pressed
			delay();
			//printf("A\n");
			// Construct the message:
			message[0] = 'A';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}


		//Make all row GPIOs High state
		*pOutPutDataReg |= (0x0F << 2);
		//make R2 low(PB3)
		*pOutPutDataReg &= ~(1 << 3);

		//Read C1, is C1 low?(PB6)
		//if PB6 is high then false else true so we use not operator
		if( !(*pInPutDataReg & (1 << 6)) ){
			//key pressed
			//debouncing logic
			delay();
			//printf("4\n");
			// Construct the message:
			message[0] = '4';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C2
		if( !(*pInPutDataReg & (1 << 7)) ){
			//key pressed
			delay();
			//printf("5\n");
			// Construct the message:
			message[0] = '5';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C3
		if( !(*pInPutDataReg & (1 << 8)) ){
			//key pressed
			delay();
			//printf("6\n");
			// Construct the message:
			message[0] = '6';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C4
		if( !(*pInPutDataReg & (1 << 9)) ){
			//key pressed
			delay();
			//printf("B\n");
			// Construct the message:
			message[0] = 'B';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}

		//Make all row GPIOs High state
		*pOutPutDataReg |= (0x0F << 2);
		//make R3 low(PB4)
		*pOutPutDataReg &= ~(1 << 4);

		//Read C1, is C1 low?(PB6)
		//if PB6 is high then false else true so we use not operator
		if( !(*pInPutDataReg & (1 << 6)) ){
			//key pressed
			//debouncing logic
			delay();
			//printf("7\n");
			// Construct the message:
			message[0] = '7';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C2
		if( !(*pInPutDataReg & (1 << 7)) ){
			//key pressed
			delay();
			//printf("8\n");
			// Construct the message:
			message[0] = '8';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C3
		if( !(*pInPutDataReg & (1 << 8)) ){
			//key pressed
			delay();
			//printf("9\n");
			// Construct the message:
			message[0] = '9';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C4
		if( !(*pInPutDataReg & (1 << 9)) ){
			//key pressed
			delay();
			//printf("C\n");
			// Construct the message:
			message[0] = 'C';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}


		//Make all row GPIOs High state
		*pOutPutDataReg |= (0x0F << 2);
		//make R4 low(PB5)
		*pOutPutDataReg &= ~(1 << 5);

		//Read C1, is C1 low?(PB6)
		//if PB6 is high then false else true so we use not operator
		if( !(*pInPutDataReg & (1 << 6)) ){
			//key pressed
			//debouncing logic
			delay();
			//printf("*\n");
			// Construct the message:
			message[0] = '*';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C2
		if( !(*pInPutDataReg & (1 << 7)) ){
			//key pressed
			delay();
			//printf("0\n");
			// Construct the message:
			message[0] = '0';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C3
		if( !(*pInPutDataReg & (1 << 8)) ){
			//key pressed
			delay();
			//printf("#\n");
			// Construct the message:
			message[0] = '#';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
		//C4
		if( !(*pInPutDataReg & (1 << 9)) ){
			//key pressed
			delay();
			//printf("D\n");
			// Construct the message:
			message[0] = 'D';
			message[1] = '\r';
			message[2] = '\n';
			message[3] = '\0';

			// Send the message (hardcoded length):
			uartWriteBuf(UART1, message, 4); // Length is known (4 bytes)

		}
	}//while end

} 
