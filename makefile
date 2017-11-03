# Replace lines below with correct paths and port for you.  
CC = /Applications/microchip/xc32/v1.31/bin/xc32-gcc
HX = /Applications/microchip/xc32/v1.31/bin/xc32-bin2hex
WRITE = ~/Desktop/PIC32/PIC32quickstart/nu32utility
PORT = /dev/tty.usbserial-1d11B
LINKSCRIPT = "NU32bootloaded.ld"
RM = rm
OBJS := $(patsubst %.c, %.o,$(wildcard *.c))
HDRS := $(wildcard *.h)
PROC = 32MX795F512L
TARGET = out

# Turn the elf file into a hex file.
$(TARGET).hex : $(TARGET).elf
	@echo Creating hex file
	$(HX) $(TARGET).elf

# Link all the object files into an elf file.
$(TARGET).elf : $(OBJS)
	@echo Linking elf file
	$(CC) -mprocessor=$(PROC) -o $(TARGET).elf $(OBJS) \
              -Wl,--defsym=__MPLAB_BUILD=1,--script=$(LINKSCRIPT),-Map=$(TARGET).map

# Create an object file for each C file.
%.o : %.c $(HDRS)
	@echo Creating object file $@
	$(CC) -g -x c -c -mprocessor=$(PROC) -o $@ $(patsubst %.o, %.c, $@)

# Erase all hex, map, object, and elf files.
clean :
	$(RM) *.hex *.map *.o *.elf        

# After making, call the NU32utility to program via bootloader.
write : $(TARGET).hex 
	$(WRITE) $(PORT) $(TARGET).hex 


