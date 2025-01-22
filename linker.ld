MEMORY
{

	FLASH(RX) : ORIGIN = 0x08000000, LENGTH = 128K
	SRAM(RWX) : ORIGIN = 0x20000000, LENGTH = 16K

}

SECTIONS{

	.text : {
		*(.isr_vector)
		*(.text)
		*(.rodata)
		. = ALIGN(4);
		_etext = .;
	} > FLASH

	.data : {
		_sdata = .;
		*(.data)
		_edata = .;
	} > SRAM AT >  FLASH
		
	.bss : {
		_sbss = .;
		*(.bss)
		_ebss = .;
	} > SRAM

}

