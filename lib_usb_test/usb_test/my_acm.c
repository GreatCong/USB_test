/*
**  USB ACm Implement 2017-10-29-by-lcj
*/
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "my_acm.h"

#define DEBUG 0 //debug

struct libusb_device_handle *devh = NULL;//设备驱动的句柄

//调用lsusb命令可以查看设备的信息
static int ep_in_addr  = 0x81;
static int ep_out_addr = 0x01;

const char * libusb_error_name(int error_code)
{
    enum libusb_error error = (libusb_error)error_code;
    switch (error) {
    case LIBUSB_SUCCESS:
        return "LIBUSB_SUCCESS";
    case LIBUSB_ERROR_IO:
        return "LIBUSB_ERROR_IO";
    case LIBUSB_ERROR_INVALID_PARAM:
        return "LIBUSB_ERROR_INVALID_PARAM";
    case LIBUSB_ERROR_ACCESS:
        return "LIBUSB_ERROR_ACCESS";
    case LIBUSB_ERROR_NO_DEVICE:
        return "LIBUSB_ERROR_NO_DEVICE";
    case LIBUSB_ERROR_NOT_FOUND:
        return "LIBUSB_ERROR_NOT_FOUND";
    case LIBUSB_ERROR_BUSY:
        return "LIBUSB_ERROR_BUSY";
    case LIBUSB_ERROR_TIMEOUT:
        return "LIBUSB_ERROR_TIMEOUT";
    case LIBUSB_ERROR_OVERFLOW:
        return "LIBUSB_ERROR_OVERFLOW";
    case LIBUSB_ERROR_PIPE:
        return "LIBUSB_ERROR_PIPE";
    case LIBUSB_ERROR_INTERRUPTED:
        return "LIBUSB_ERROR_INTERRUPTED";
    case LIBUSB_ERROR_NO_MEM:
        return "LIBUSB_ERROR_NO_MEM";
    case LIBUSB_ERROR_NOT_SUPPORTED:
        return "LIBUSB_ERROR_NOT_SUPPORTED";
    case LIBUSB_ERROR_OTHER:
        return "LIBUSB_ERROR_OTHER";
    }
    return "**UNKNOWN**";
}


void write_char(unsigned char c)
{
    /* To send a char to the device simply initiate a bulk_transfer to the
     * Endpoint with address ep_out_addr.
     */
    int actual_length;
    if (libusb_bulk_transfer(devh, ep_out_addr, &c, 1,
                             &actual_length, 0) < 0) {
        fprintf(stderr, "Error while sending char\n");
    }
}

int read_chars(unsigned char * data, int size)
{
    /* To receive characters from the device initiate a bulk_transfer to the
     * Endpoint with address ep_in_addr.
     */
    int actual_length;
    int rc = libusb_bulk_transfer(devh, ep_in_addr, data, size, &actual_length,10);
    if (rc == LIBUSB_ERROR_TIMEOUT) {

        #if DEBUG
          printf("timeout (%d)\n", actual_length);
        #endif

        return -1;
    } else if (rc < 0) {
        fprintf(stderr, "Error while waiting for char:%s\n", libusb_error_name(rc));
        return -1;
    }

    return actual_length;
}

int USB_CDC_init(uint16_t vendor_id,uint16_t product_id){
    int rc;
    // uint8_t request_type = 0x21;
    // uint8_t request_value = 0x22;
    uint8_t request_type = USB_RT_ACM;
    uint8_t request_value = ACM_REQ_RESPONSE;
     /* - set line encoding: here 9600 8N1
     * 9600 = 0x2580 ~> 0x80, 0x25 in little endian
     */
    unsigned char encoding[] = { 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 };

    /* Initialize libusb
     */
    system("mount -t usbdevfs none /proc/bus/usb");//分配内存
    rc = libusb_init(NULL);
    if (rc < 0) {
        fprintf(stderr, "Error initializing libusb: %s\n", libusb_error_name(rc));
        printf("rc=%d ",rc);
        exit(1);
    }

    /* Set debugging output to max level.
     */
    libusb_set_debug(NULL, 3);

    /* Look for a specific device and open it.
     */
    devh = libusb_open_device_with_vid_pid(NULL, vendor_id, product_id);
    if (!devh) {
        fprintf(stderr, "Error finding USB device\n");
        // goto out;
        return -1;
    }

    
    for (int if_num = 0; if_num < 2; if_num++) { 
        if (libusb_kernel_driver_active(devh, if_num)) { //如果插入的设备已经被内核调用，需要将其与内核断开
            printf("doing detach_kernel_driver\n");
            libusb_detach_kernel_driver(devh, if_num);
        }
        rc = libusb_claim_interface(devh, if_num);
        if (rc < 0) {
            fprintf(stderr, "Error claiming interface:%s\n", libusb_error_name(rc));
            // goto out;
            return -1;
        }
    }

    //ACM设备的初始化设置
    rc = libusb_control_transfer(devh, request_type, request_value, ACM_CTRL_DTR | ACM_CTRL_RTS,0, NULL, 0, 0);
    if (rc < 0) {
        fprintf(stderr, "Error during control transfer:%s\n", libusb_error_name(rc));
    }

    /* - set line encoding: here 9600 8N1
     * 9600 = 0x2580 ~> 0x80, 0x25 in little endian 不过对于CDC来说这个不重要
     */
    // unsigned char encoding[] = { 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 };
    rc = libusb_control_transfer(devh, request_type, request_value, ACM_CTRL_DTR | ACM_CTRL_RTS, 0, encoding,
                                sizeof(encoding), 0);
    if (rc < 0) {
        fprintf(stderr, "Error during control transfer: %s\n", libusb_error_name(rc));
    }

    return rc;
}