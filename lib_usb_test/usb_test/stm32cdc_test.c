/*
 * STM32 VCP ACM简单测试 ----2017.10.27--by-lcj
 */
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>

#include "my_acm.h"
#include "libusb.h"

#define VENDOR_ID      0x0483   // STM32 
#define PRODUCT_ID     0x5740   // STM32

// //signal handle
// static void sighandler(int signum){
//    // if (devh)
//    //          libusb_close(devh);
//     //libusb_exit(NULL);
//     printf("usb is closed\n");
// }

int main(int argc, char **argv)
{
    
    unsigned char read_buf[65]={0};//缓冲区
    int read_len = 0;
    int rc;

    rc=USB_CDC_init(VENDOR_ID, PRODUCT_ID);
    if(rc<0)
        goto out;

    printf("read start\n");  
    while(1) {
        //write_char('t');
        read_len = read_chars(read_buf, 10);
        // printf("len=%d\n", read_len);
        if(read_len>0){
            read_buf[read_len] = '\0';
            fprintf(stdout, "Received[%d]: \"%s\"\n", read_len,read_buf);
        }
        else{
            printf("no data\n");
        }

        sleep(1);//delay 1s
         // usleep(3000);//delay 3ms
        // signal(SIGINT,sighandler);
    }

    libusb_release_interface(devh, 0);//release the interface

out:
    if (devh)
            libusb_close(devh);
    libusb_exit(NULL);
    printf("usb is closed\n");
    return rc;
}
