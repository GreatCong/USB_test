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
         
all:libmyUSB.so
SRC=ftdi.o usb.o

ftdi.o:
	g++ -c ftdi.c -o ftdi.o
usb.o:
	g++ -c usb.c -o usb.o
libmyUSB.so:$(SRC)
	g++ -fPIC -shared -o $@ $(SRC) -lpthread -lrt -ldl &&rm *.o
clean:
	rm *.o *.so