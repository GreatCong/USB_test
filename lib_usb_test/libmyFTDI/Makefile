all:libmyFTDI.so
SRC=ftdi.o ftdi_stream.o sync.o core.o linux_usbfs.o descriptor.o io.o
#include ./ current path
INC = -I./
CC = g++

ftdi.o:
	$(CC) $(INC) -c ftdi.c -o ftdi.o
ftdi_stream.o:
	$(CC) $(INC) -c ftdi_stream.c -o ftdi_stream.o
sync.o:
	$(CC) $(INC) -c sync.c -o sync.o
core.o:
	$(CC) $(INC) -c core.c -o core.o 
linux_usbfs.o:
	$(CC) $(INC) -c linux_usbfs.c -o linux_usbfs.o
descriptor.o:
	$(CC) $(INC) -c descriptor.c -o descriptor.o
io.o:
	$(CC) $(INC) -c io.c -o io.o
libmyFTDI.so:$(SRC)
	$(CC) -fPIC -shared -o $@ $(SRC) -lpthread -lrt -ldl &&rm *.o
clean:
	rm *.o *.so