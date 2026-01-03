# Toolchain
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS = -Wall -O2 -mcpu=cortex-m4 -mthumb -nostdlib -nostartfiles \
         -Iinc -Isystem
LDFLAGS = -Tlinker/linker.ld

# Files
SRC_C = src/main.c system_stm32f4xx.c src/libc_stub.c src/lib/delays.c src/lib/uart.c src/lib/ov7670_stm32.c src/lib/ov7670.c
SRC_S = startup_stm32f401xe.s
OBJ = $(SRC_C:.c=.o) $(SRC_S:.s=.o)

# Targets
all: main.elf main.bin

# Compile C files
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Compile assembly file using arm-none-eabi-gcc
%.o: %.s
	$(CC) -c -mcpu=cortex-m4 -mthumb -o $@ $<

# Link the ELF file
main.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

# Convert ELF to binary
main.bin: main.elf
	$(OBJCOPY) -O binary $< $@

# Flash binary to board
flash: main.bin
	st-flash write main.bin 0x8000000

# Clean build artifacts
clean:
	rm -f src/*.o system/*.o startup/*.o *.elf *.bin
