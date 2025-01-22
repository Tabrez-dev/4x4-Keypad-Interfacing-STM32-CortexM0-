#include<stdint.h>



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
	}
//C2
if( !(*pInPutDataReg & (1 << 7)) ){
		//key pressed
		delay();
		//printf("2\n");
	}
//C3
if( !(*pInPutDataReg & (1 << 8)) ){
		//key pressed
		delay();
		//printf("3\n");
	}
//C4
if( !(*pInPutDataReg & (1 << 9)) ){
		//key pressed
		delay();
		//printf("A\n");
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
	}
//C2
if( !(*pInPutDataReg & (1 << 7)) ){
		//key pressed
		delay();
		//printf("5\n");
	}
//C3
if( !(*pInPutDataReg & (1 << 8)) ){
		//key pressed
		delay();
		//printf("6\n");
	}
//C4
if( !(*pInPutDataReg & (1 << 9)) ){
		//key pressed
		delay();
		//printf("B\n");
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
	}
//C2
if( !(*pInPutDataReg & (1 << 7)) ){
		//key pressed
		delay();
		//printf("8\n");
	}
//C3
if( !(*pInPutDataReg & (1 << 8)) ){
		//key pressed
		delay();
		//printf("9\n");
	}
//C4
if( !(*pInPutDataReg & (1 << 9)) ){
		//key pressed
		delay();
		//printf("C\n");
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
	}
//C2
if( !(*pInPutDataReg & (1 << 7)) ){
		//key pressed
		delay();
		//printf("0\n");
	}
//C3
if( !(*pInPutDataReg & (1 << 8)) ){
		//key pressed
		delay();
		//printf("#\n");
	}
//C4
if( !(*pInPutDataReg & (1 << 9)) ){
		//key pressed
		delay();
		//printf("D\n");
	}
}//while end






} 
