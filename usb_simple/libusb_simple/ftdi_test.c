#include <stdio.h>
#include <stdlib.h>
#include "ftdi.h"
#include <string.h>
#include <unistd.h>
int open_test() {
  int r = 0;
  while (!(r = ftdi_open(0x0403, 0x6001))) {
    sleep(1);
  }
  return r;
}
// void read_test() {
//   char read_buf[128] = {'\0'};
//   while (1) {
//     int ret = ftdi_read_data(read_buf, 16);
//     if (ret > 0) {
//       printf("read size:%d\n", ret);
//       read_buf[ret] = '\0';
//       printf("receive:%s\n", read_buf);
//     }
//   };
// }
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
    open_test();
    printf("OK\n");
  }
  ftdi_close();
}
