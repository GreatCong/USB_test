#include <stdio.h>
#include <stdlib.h>
#include "ftdi.h"
#include <string.h>
#include <unistd.h>

// #include "bluetooth.h"
#define MY_VEND 0x0403
#define MY_PROD 0x6001
// #define MY_VEND 0x0483
// #define MY_PROD 0x5740

int open_test() {
  int r = 0;
  int n=0;
  system("mount -t usbdevfs none /proc/bus/usb");//这个函数应该是分配内存的
  while (!(r = ftdi_open(MY_VEND, MY_PROD))) {
    sleep(1);
  }
  // r = ftdi_open(0x0403, 0x6001);
  // printf("%d ", r);
  return r;
}
void read_test() {
  unsigned char read_buf[128] = {'\0'};
  while (1) {
    int ret = ftdi_read_data(read_buf, 16);
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

int main(void) {
  int ret = 0;
  printf("start\n");
  ret = ftdi_init();
  printf("ftdi_init:%d\n", ret);
  if (ret) {
    // write_test();
    printf("open test\n");
    ret = open_test();
    printf("OK,ret=%d\n",ret);
  }
  ftdi_set_baudrate(115200);
  read_test();
  // ret = BluetoothInit();
// printf("BluetoothInit:%d\n", ret);

  ftdi_close();
}
