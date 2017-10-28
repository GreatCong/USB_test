#ifndef FTDI_H
#define FTDI_H
#include "types.h"
#ifdef __cplusplus
extern "C" {
#endif
	int ftdi_init();
	int ftdi_open(int vid, int pid);
	void ftdi_close();
	int ftdi_write_data(const unsigned char* buf, int size);
	int ftdi_read_data(unsigned char* buf, int size);
	int ftdi_get_state();

	int ftdi_usb_reset(struct ftdi_context* ftdi) ;
	static int ftdi_to_clkbits(int baudrate,unsigned int clk,int clk_div,unsigned long* encoded_divisor);
	static int ftdi_to_clkbits_AM(int baudrate, unsigned long* encoded_divisor);
	static int ftdi_convert_baudrate(int baudrate,unsigned short* value,unsigned short* index);
	int ftdi_set_baudrate(int baudrate) ;
	int ftdi_set_line_property2(enum ftdi_bits_type bits,enum ftdi_stopbits_type sbit,
		                        enum ftdi_parity_type parity,enum ftdi_break_type break_type);
	int ftdi_set_line_property(enum ftdi_bits_type bits,enum ftdi_stopbits_type sbit,
                               enum ftdi_parity_type parity);
	int ftdi_read_chipid(unsigned int* chipid) ;
	int ftdi_usb_open_dev() ;


#ifdef __cplusplus
}
#endif
#endif
