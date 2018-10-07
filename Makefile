TRG = mbloader

SRC = $(TRG).c com.c modbus.c modbus-tcp.c  modbus-rtu.c  modbus-data.c
HD  = com.h protocol.h config.h modbus-version.h modbus.h modbus-tcp.h modbus-tcp-private.h modbus-rtu.h modbus-rtu-private.h modbus-private.h
OBJ = $(SRC:.c=.o)

CCFLAGS = -Wall -g -O3

all : $(TRG)

%.o : %.c $(HD)
	gcc $(CCFLAGS) -c $< -o $@

$(TRG) : $(OBJ)
	gcc $(CCFLAGS) $(OBJ) -o $@

clean:
	rm -f $(OBJ)
	#rm -f $(TRG)
