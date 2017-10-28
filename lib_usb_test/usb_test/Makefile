# all:test
# # SRC=ftdi.o usb.o
# test:ftdi.o usb.o ftdi_test.o
# 	g++ -o ftdi.o usb.o ftdi_test.o -o test
# ftdi.o:
# 	g++ ftdi.c -o ftdi
# usb.o:
# 	g++ usb.c -o usb
# ftdi_test.o:
# 	g++ ftdi_test.c -o ftdi_test

# clean:
# 	rm *.o *.so
         
all:test

INC= -I./inc_so/
CC = g++
LFLAGS = libmyUSB.so -L.-lmyUSB
SRC = stm32cdc_test.c

test:$(SRC) #添加依赖项
	$(CC) $(INC) $(SRC) $(LFLAGS) -o test

clean:
	rm -rf *.o *.out test