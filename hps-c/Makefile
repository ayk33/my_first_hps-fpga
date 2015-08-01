#
TARGET = my_first_hps-fpga
TARGET_DIR = bin
SRC_DIR = src

program_C_SRCS :=  $(wildcard src/*.c)
program_C_OBJS := $(program_C_SRCS:.c=.o)
program_INCLUDE_DIRS :=  inc ${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include 

#



CROSS_COMPILE = arm-linux-gnueabihf-

#CFLAGS = -static -g -Wall 
CFLAGS += $(foreach includedir,$(program_INCLUDE_DIRS),-I$(includedir))
LDFLAGS =  -g -Wall  
CC = $(CROSS_COMPILE)gcc
ARCH= arm


all: $(TARGET_DIR)/$(TARGET)

$(TARGET_DIR)/$(TARGET) : $(program_C_OBJS)
	$(CC) $(LDFLAGS) $(program_C_OBJS) -o $(TARGET_DIR)/$(TARGET)


.PHONY: clean
clean:
	rm -f $(TARGET_DIR)/$(TARGET) *.a *.o *~ 
	rm -f $(program_C_OBJS) *.a *.o *~ 