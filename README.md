# 4x4 Keypad Interfacing on STM32F072RB-DISCOVERY

This project demonstrates the interfacing of a 4x4 keypad with the STM32F072RB microcontroller, using direct memory access and register manipulation. The main aim of this project is to read keypresses from a 4x4 matrix keypad and process the inputs accordingly.

## Key Features

- STM32F072RB-DISCOVERY board
- Interfacing with a 4x4 matrix keypad
- Direct access to memory-mapped I/O registers
- Custom reset handler for memory initialization
- Clock configuration through RCC registers
- Simple debouncing for key presses using delay

## Technical Details

1. **Memory Layout:**
   - SRAM: 16KB starting from address `0x20000000`
   - Flash: 128KB starting from address `0x08000000`
   
2. **Sections:**
   - **Text Section:** Contains interrupt vectors and code (.text, .isr_vector, .rodata)
   - **Data Section:** Used for initialized variables, copied from Flash to SRAM on boot
   - **BSS Section:** Uninitialized variables, initialized to zero during startup

3. **System Clock Configuration:**
   - RCC (Reset and Clock Control) registers are configured for system clock setup.
   - The processor operates at the default 8 MHz frequency, using the HSI (High-Speed Internal) oscillator.

4. **Interrupt Vector Table:**
   - The vector table is placed at the top of SRAM with the reset handler as the first entry.
   
5. **Keypad Interfacing:**
   - GPIO pins are configured as input and output for the rows and columns of the keypad.
   - Internal pull-up resistors are enabled for the column pins to detect keypresses.
   - A simple scanning algorithm is used to detect which key has been pressed based on the row and column combination.

## Pin Configuration

- **GPIOB Pins:**
  - Pins PB2 to PB5: Configured as output for the rows of the keypad.
  - Pins PB6 to PB9: Configured as input with pull-up resistors for the columns.

## Usage

1. **Build:**
   Compile and link the project using the provided `Makefile`.

2. **Programming:**
   Program the compiled `.elf` file onto the STM32F072RB-DISCOVERY board using an appropriate flashing tool like OpenOCD or ST-Link.

3. **Operation:**
   The system waits for key presses on the 4x4 keypad and processes the input when a key is pressed. The result can be displayed or used in further application logic.

## Project Structure

- `main.c`: Contains the main application code, including initialization, keypad scanning, and system configuration.
- `linker.ld`: Defines memory layout and linker script.
- `Makefile`: Used to build the project with GCC for ARM.
- `.gitignore`: Ignores build files and other unnecessary files for version control.

## Work Remaining

- **UART-based `printf`:** The `printf` function for debugging is currently commented out as it is not yet implemented for this microcontroller. The UART will be configured, and `printf` functionality will be added in a future update.(Added but yet to be tested)


