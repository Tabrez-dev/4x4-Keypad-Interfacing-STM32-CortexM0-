# Compiler and Tools
CC      := arm-none-eabi-gcc
LD      := arm-none-eabi-ld
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
READELF := arm-none-eabi-readelf
NM      := arm-none-eabi-nm

# Compiler and Linker Flags
CFLAGS  := -O0 -mthumb -mcpu=cortex-m0 -c
LDFLAGS := -Map=main.map -Tlinker.ld

# Source and Output Files
SRC     := main.c
OBJ     := main.o
ELF     := main.elf
BIN     := main.bin
MAP     := main.map
DEBUG   := main.debug
NM_OUT  := main.nm
DISASM  := main.disasm

# Phony targets (prevent conflicts with actual files)
.PHONY: all clean

# Default target
all: $(ELF) $(BIN) $(DEBUG) $(NM_OUT) $(DISASM)

# Compilation
$(OBJ): $(SRC)
	$(CC) $(CFLAGS) $< -o $@

# Linking
$(ELF): $(OBJ)
	$(LD) $(LDFLAGS) $< -o $@

# Create debug information
$(DEBUG): $(ELF)
	$(READELF) -a $< > $@

# Create binary output
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

# Generate symbol table
$(NM_OUT): $(ELF)
	$(NM) $< > $@

# Generate disassembly
$(DISASM): $(ELF)
	$(OBJDUMP) -d $< > $@

# Clean build artifacts
clean:
	rm -rf $(OBJ) $(ELF) $(BIN) $(MAP) $(DEBUG) $(NM_OUT) $(DISASM)

