/* 
*ftdi uart test using libftdi --2017.10.27--by-lcj
*/

#include <stdio.h>
#include <stdlib.h>
#include <ftdi.h>

void read_test(struct ftdi_context *ftdi) {
  unsigned char read_buf[128] = {'\0'};
  while (1) {
    int ret = ftdi_read_data(ftdi,read_buf, 16);
    if (ret > 0) {
      printf("read size:%d ", ret);
      read_buf[ret] = '\0';
      printf("receive:%s\n", read_buf);
    }
  }
}
// void write_test() {
//   char* write_buf = "abcdefghijk";
// open:
//   printf("open device\n");
//   open_test();
//   while (1) {
//     int ret = ftdi_get_state();
//     if (ret) {
//       ret = ftdi_write_data(write_buf, strlen(write_buf));
//       printf("write size:%d\n", ret);
//       sleep(2);
//     } else {
//       goto open;
//     }
//   };
// }

int main(void)
{
    int ret;
    struct ftdi_context *ftdi;
    struct ftdi_version_info version;
    system("mount -t usbdevfs none /proc/bus/usb");//这个函数应该是分配内存的
    if ((ftdi = ftdi_new()) == 0)
   {
        fprintf(stderr, "ftdi_new failed\n");
        return EXIT_FAILURE;
    }

    version = ftdi_get_library_version();
    printf("Initialized libftdi %s (major: %d, minor: %d, micro: %d, snapshot ver: %s)\n",
        version.version_str, version.major, version.minor, version.micro,
        version.snapshot_str);

    if ((ret = ftdi_usb_open(ftdi, 0x0403, 0x6001)) < 0)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    // Read out FTDIChip-ID of R type chips
    if (ftdi->type == TYPE_R)
    {
        unsigned int chipid;
        printf("ftdi_read_chipid: %d\n", ftdi_read_chipid(ftdi, &chipid));
        printf("FTDI chipid: %X\n", chipid);
    }

    // Set baudrate
    ret = ftdi_set_baudrate(ftdi, 115200);
    if (ret < 0)
    {
        fprintf(stderr, "unable to set baudrate: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        exit(-1);
    }
    
    /* Set line parameters */
    ret = ftdi_set_line_property(ftdi, BITS_8, STOP_BIT_1, NONE);
    if (ret < 0)
    {
        fprintf(stderr, "unable to set line parameters: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        exit(-1);
    }

    read_test(ftdi);

    if ((ret = ftdi_usb_close(ftdi)) < 0)
    {
        fprintf(stderr, "unable to close ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    ftdi_free(ftdi);

    return EXIT_SUCCESS;
}
