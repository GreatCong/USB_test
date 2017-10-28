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
test:
	g++ ftdi_test.c libmyUSB.so -lmyUSB

clean:
	rm *.o *.out