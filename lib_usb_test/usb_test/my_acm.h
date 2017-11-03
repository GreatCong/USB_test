/*
**  USB ACm Header 2017-10-29-by-lcj
*/
#ifndef __MY_ACM_H__
#define __MY_ACM_H__

#include "libusb.h"

/*
 * Major and minor numbers.
 */
#define ACM_TTY_MAJOR   166
#define ACM_TTY_MINORS  32
/*
 * Requests.
 */
#define USB_RT_ACM  (LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE)
#define ACM_REQ_COMMAND        0x00
#define ACM_REQ_RESPONSE       0x01
#define ACM_REQ_SET_FEATURE    0x02
#define ACM_REQ_GET_FEATURE    0x03
#define ACM_REQ_CLEAR_FEATURE  0x04
#define ACM_REQ_SET_LINE       0x20
#define ACM_REQ_GET_LINE       0x21
#define ACM_REQ_SET_CONTROL    0x22
#define ACM_REQ_SEND_BREAK     0x23
/*
 * IRQs.
 */
#define ACM_IRQ_NETWORK 0x00
#define ACM_IRQ_LINE_STATE  0x20
/*
 * Output control lines.
 */
#define ACM_CTRL_DTR    0x01
#define ACM_CTRL_RTS    0x02
/*
 * Input control lines and line errors.
 */
#define ACM_CTRL_DCD    0x01
#define ACM_CTRL_DSR    0x02
#define ACM_CTRL_BRK    0x04
#define ACM_CTRL_RI     0x08
#define ACM_CTRL_FRAMING    0x10
#define ACM_CTRL_PARITY     0x20
#define ACM_CTRL_OVERRUN    0x40

const char * libusb_error_name(int error_code);
void write_char(unsigned char c);
int read_chars(unsigned char * data, int size);
int USB_CDC_init(uint16_t vendor_id,uint16_t product_id);

extern struct libusb_device_handle *devh;

#endif
