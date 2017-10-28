/*
*	硬件读写相关函数
*	修改自libftdi,libftdi开源地址：https://www.intra2net.com/en/developer/libftdi/index.php
*	在源码基础上删除了所有未用到的功能和函数，增加了部分函数；根据需要，修改了所有动态内
*	存相关函数，所有动态内存都改为蓝牙模块初始化时分配，运行过程中不再动态分配内存；修改
*	了部分函数名；整合了多个源文件
*/
#include "types.h"
#include "usb.h"
#include <stdlib.h>
#include <stdio.h>
#include "ftdi.h"
#include <string.h>
#define BAUD_RATE 1382400
struct ftdi_context* ftdi_ctx;
struct ftdi_eeprom* ftdi_eep;
unsigned char* ftdi_read_buf;
static unsigned char ftdi_read_chipid_shift(unsigned char value) {
  return ((value & 1) << 1) | ((value & 2) << 5) | ((value & 4) >> 2) |
         ((value & 8) << 4) | ((value & 16) >> 1) | ((value & 32) >> 1) |
         ((value & 64) >> 4) | ((value & 128) >> 2);
}
int ftdi_init(void) {
  int r = 0;
  ftdi_ctx = (struct ftdi_context*)calloc(sizeof(struct ftdi_context), 1);
  if (!ftdi_ctx) {
    r = -1;
    goto out;
  }
  ftdi_eep = (struct ftdi_eeprom*)calloc(sizeof(struct ftdi_eeprom), 1);
  if (!ftdi_eep) {
    r = -2;
    goto i1;
  }
  ftdi_read_buf = (unsigned char*)calloc(4096, 1);
  if (!ftdi_read_buf || usb_mem_init() < 0) {
    r = -3;
    goto i2;
  }

  ftdi_ctx->usb_ctx = NULL;
  ftdi_ctx->usb_dev = NULL;
  ftdi_ctx->usb_read_timeout = 1000;
  ftdi_ctx->usb_write_timeout = 1000;

  ftdi_ctx->type = TYPE_BM;
  ftdi_ctx->baudrate = BAUD_RATE;
  ftdi_ctx->bitbang_enabled = 0;

  ftdi_ctx->readbuffer = NULL;
  ftdi_ctx->readbuffer_offset = 0;
  ftdi_ctx->readbuffer_remaining = 0;
  ftdi_ctx->writebuffer_chunksize = 40960;
  ftdi_ctx->max_packet_size = 0;
  ftdi_ctx->error_str = NULL;

  ftdi_ctx->interface = 0;
  ftdi_ctx->index = INTERFACE_A;
  ftdi_ctx->in_ep = 0x02;
  ftdi_ctx->out_ep = 0x81;
  ftdi_ctx->usb_ctx = usb_ctx;
  ftdi_ctx->bitbang_mode = 1;
  ftdi_ctx->eeprom = ftdi_eep;
  ftdi_ctx->readbuffer_offset = 0;
  ftdi_ctx->readbuffer_remaining = 0;
  ftdi_ctx->readbuffer = ftdi_read_buf;
  ftdi_ctx->readbuffer_chunksize = 4096;
  return 1;
  FREE(ftdi_read_buf);
i2:
  FREE(ftdi_eep);
i1:
  FREE(ftdi_ctx);
out:
  return r;
}
int ftdi_usb_reset(struct ftdi_context* ftdi) {
  if (control_transfer_func(FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST,
                            SIO_RESET_SIO, ftdi->index, NULL, 0,
                            ftdi->usb_write_timeout) < 0)
    return -1;
  ftdi->readbuffer_offset = 0;
  ftdi->readbuffer_remaining = 0;

  return 0;
}
static unsigned int _ftdi_determine_max_packet_size() {
  struct usb_device_descriptor desc;
  struct usb_config_descriptor config0;
  unsigned int packet_size;
  if (ftdi_ctx->type == TYPE_2232H || ftdi_ctx->type == TYPE_4232H ||
      ftdi_ctx->type == TYPE_232H)
    packet_size = 512;
  else
    packet_size = 64;

  if (get_device_descriptor(&desc) < 0) {
    return packet_size;
  }

  if (get_config_descriptor(0, &config0) < 0) {
    return packet_size;
  }
  if (desc.bNumConfigurations > 0) {
    if (ftdi_ctx->interface < config0.bNumInterfaces) {
      struct usb_interface interface = config0.interface[ftdi_ctx->interface];
      if (interface.num_altsetting > 0) {
        struct usb_interface_descriptor descriptor = interface.altsetting[0];
        if (descriptor.bNumEndpoints > 0) {
          packet_size = descriptor.endpoint[0].wMaxPacketSize;
        }
      }
    }
  }
  return packet_size;
}
static int ftdi_to_clkbits(int baudrate,
                           unsigned int clk,
                           int clk_div,
                           unsigned long* encoded_divisor) {
  static const char frac_code[8] = {0, 3, 2, 4, 1, 5, 6, 7};
  int best_baud = 0;
  int divisor, best_divisor;
  if (baudrate >= clk / clk_div) {
    *encoded_divisor = 0;
    best_baud = clk / clk_div;
  } else if (baudrate >= clk / (clk_div + clk_div / 2)) {
    *encoded_divisor = 1;
    best_baud = clk / (clk_div + clk_div / 2);
  } else if (baudrate >= clk / (2 * clk_div)) {
    *encoded_divisor = 2;
    best_baud = clk / (2 * clk_div);
  } else {
    divisor = clk * 16 / clk_div / baudrate;
    if (divisor & 1) 
      best_divisor = divisor / 2 + 1;
    else
      best_divisor = divisor / 2;
    if (best_divisor > 0x20000)
      best_divisor = 0x1ffff;
    best_baud = clk * 16 / clk_div / best_divisor;
    if (best_baud & 1)
      best_baud = best_baud / 2 + 1;
    else
      best_baud = best_baud / 2;
    *encoded_divisor =
        (best_divisor >> 3) | (frac_code[best_divisor & 0x7] << 14);
  }
  return best_baud;
}
static int ftdi_to_clkbits_AM(int baudrate, unsigned long* encoded_divisor)

{
  static const char frac_code[8] = {0, 3, 2, 4, 1, 5, 6, 7};
  static const char am_adjust_up[8] = {0, 0, 0, 1, 0, 3, 2, 1};
  static const char am_adjust_dn[8] = {0, 0, 0, 1, 0, 1, 2, 3};
  int divisor, best_divisor, best_baud, best_baud_diff;
  int i;
  divisor = 24000000 / baudrate;

  divisor -= am_adjust_dn[divisor & 7];

  best_divisor = 0;
  best_baud = 0;
  best_baud_diff = 0;
  for (i = 0; i < 2; i++) {
    int try_divisor = divisor + i;
    int baud_estimate;
    int baud_diff;

    if (try_divisor <= 8) {
      try_divisor = 8;
    } else if (divisor < 16) {
      try_divisor = 16;
    } else {
      try_divisor += am_adjust_up[try_divisor & 7];
      if (try_divisor > 0x1FFF8) {
        try_divisor = 0x1FFF8;
      }
    }
    baud_estimate = (24000000 + (try_divisor / 2)) / try_divisor;
    if (baud_estimate < baudrate) {
      baud_diff = baudrate - baud_estimate;
    } else {
      baud_diff = baud_estimate - baudrate;
    }
    if (i == 0 || baud_diff < best_baud_diff) {
      best_divisor = try_divisor;
      best_baud = baud_estimate;
      best_baud_diff = baud_diff;
      if (baud_diff == 0) {
        break;
      }
    }
  }
  *encoded_divisor = (best_divisor >> 3) | (frac_code[best_divisor & 7] << 14);
  if (*encoded_divisor == 1) {
    *encoded_divisor = 0;  
  } else if (*encoded_divisor == 0x4001) {
    *encoded_divisor = 1;  
  }
  return best_baud;
}
static int ftdi_convert_baudrate(int baudrate,
                                 unsigned short* value,
                                 unsigned short* index) {
  int best_baud;
  unsigned long encoded_divisor;

  if (baudrate <= 0) {
    return -1;
  }

#define H_CLK 120000000
#define C_CLK 48000000
  if ((ftdi_ctx->type == TYPE_2232H) || (ftdi_ctx->type == TYPE_4232H) ||
      (ftdi_ctx->type == TYPE_232H)) {
    if (baudrate * 10 > H_CLK / 0x3fff) {
      best_baud = ftdi_to_clkbits(baudrate, H_CLK, 10, &encoded_divisor);
      encoded_divisor |= 0x20000; 
    } else
      best_baud = ftdi_to_clkbits(baudrate, C_CLK, 16, &encoded_divisor);
  } else if ((ftdi_ctx->type == TYPE_BM) || (ftdi_ctx->type == TYPE_2232C) ||
             (ftdi_ctx->type == TYPE_R)) {
    best_baud = ftdi_to_clkbits(baudrate, C_CLK, 16, &encoded_divisor);
  } else {
    best_baud = ftdi_to_clkbits_AM(baudrate, &encoded_divisor);
  }
  *value = (unsigned short)(encoded_divisor & 0xFFFF);
  if (ftdi_ctx->type == TYPE_2232H || ftdi_ctx->type == TYPE_4232H ||
      ftdi_ctx->type == TYPE_232H) {
    *index = (unsigned short)(encoded_divisor >> 8);
    *index &= 0xFF00;
    *index |= ftdi_ctx->index;
  } else
    *index = (unsigned short)(encoded_divisor >> 16);
  return best_baud;
}

int ftdi_set_baudrate(int baudrate) {
  unsigned short value, index;
  int actual_baudrate;

  if (ftdi_ctx->bitbang_enabled) {
    baudrate = baudrate * 4;
  }

  actual_baudrate = ftdi_convert_baudrate(baudrate, &value, &index);
  if (actual_baudrate <= 0) {
    return -1;
  }
  if ((actual_baudrate * 2 < baudrate) ||
      ((actual_baudrate < baudrate) ? (actual_baudrate * 21 < baudrate * 20)
                                    : (baudrate * 21 < actual_baudrate * 20))) {
    return -2;
  }

  if (control_transfer_func(FTDI_DEVICE_OUT_REQTYPE, SIO_SET_BAUDRATE_REQUEST,
                            value, index, NULL, 0,
                            ftdi_ctx->usb_write_timeout) < 0) {
    return -3;
  }

  ftdi_ctx->baudrate = baudrate;
  return 0;
}
int ftdi_set_line_property2(enum ftdi_bits_type bits,
                            enum ftdi_stopbits_type sbit,
                            enum ftdi_parity_type parity,
                            enum ftdi_break_type break_type) {
  unsigned short value = bits;
  switch (parity) {
    case NONE:
      value |= (0x00 << 8);
      break;
    case ODD:
      value |= (0x01 << 8);
      break;
    case EVEN:
      value |= (0x02 << 8);
      break;
    case MARK:
      value |= (0x03 << 8);
      break;
    case SPACE:
      value |= (0x04 << 8);
      break;
  }

  switch (sbit) {
    case STOP_BIT_1:
      value |= (0x00 << 11);
      break;
    case STOP_BIT_15:
      value |= (0x01 << 11);
      break;
    case STOP_BIT_2:
      value |= (0x02 << 11);
      break;
  }

  switch (break_type) {
    case BREAK_OFF:
      value |= (0x00 << 14);
      break;
    case BREAK_ON:
      value |= (0x01 << 14);
      break;
  }

  if (control_transfer_func(FTDI_DEVICE_OUT_REQTYPE, SIO_SET_DATA_REQUEST,
                            value, ftdi_ctx->index, NULL, 0,
                            ftdi_ctx->usb_write_timeout) < 0) {
    return -1;
  }
  return 0;
}
int ftdi_set_line_property(enum ftdi_bits_type bits,
                           enum ftdi_stopbits_type sbit,
                           enum ftdi_parity_type parity) {
  return ftdi_set_line_property2(bits, sbit, parity, BREAK_OFF);
}
int ftdi_read_chipid(unsigned int* chipid) {
  unsigned int a = 0, b = 0;
  if (control_transfer_func(FTDI_DEVICE_IN_REQTYPE, SIO_READ_EEPROM_REQUEST, 0,
                            0x43, (unsigned char*)&a, 2,
                            ftdi_ctx->usb_read_timeout) == 2) {
    a = a << 8 | a >> 8;
    if (control_transfer_func(FTDI_DEVICE_IN_REQTYPE, SIO_READ_EEPROM_REQUEST,
                              0, 0x44, (unsigned char*)&b, 2,
                              ftdi_ctx->usb_read_timeout) == 2) {
      b = b << 8 | b >> 8;
      a = (a << 16) | (b & 0xFFFF);
      a = ftdi_read_chipid_shift(a) | ftdi_read_chipid_shift(a >> 8) << 8 |
          ftdi_read_chipid_shift(a >> 16) << 16 |
          ftdi_read_chipid_shift(a >> 24) << 24;
      *chipid = a ^ 0xa5f0f7d1;
      return 0;
    }
  }
  return -1;
}
int ftdi_read_data(unsigned char* buf, int size) {
  int offset = 0, ret, i, num_of_chunks, chunk_remains;
  int packet_size = ftdi_ctx->max_packet_size;
  int actual_length = 1;

  if (packet_size == 0) {
    return -1;
  }
  if (size <= (int)ftdi_ctx->readbuffer_remaining) {
    memcpy(buf, ftdi_ctx->readbuffer + ftdi_ctx->readbuffer_offset, size);
    ftdi_ctx->readbuffer_remaining -= size;
    ftdi_ctx->readbuffer_offset += size;
    return size;
  }
  if (ftdi_ctx->readbuffer_remaining != 0) {
    memcpy(buf, ftdi_ctx->readbuffer + ftdi_ctx->readbuffer_offset,
           ftdi_ctx->readbuffer_remaining);
    offset += ftdi_ctx->readbuffer_remaining;
  }
  while (offset < size && actual_length > 0) {
    ftdi_ctx->readbuffer_remaining = 0;
    ftdi_ctx->readbuffer_offset = 0;
    ret = bulk_transfer_func(ftdi_ctx->out_ep, ftdi_ctx->readbuffer,
                             ftdi_ctx->readbuffer_chunksize, &actual_length,
                             ftdi_ctx->usb_read_timeout);
    if (ret < 0) {
      return -2;
    }
    if (actual_length > 2) {
      num_of_chunks = actual_length / packet_size;
      chunk_remains = actual_length % packet_size;
      ftdi_ctx->readbuffer_offset += 2;
      actual_length -= 2;

      if (actual_length > packet_size - 2) {
        for (i = 1; i < num_of_chunks; i++)
          memmove(ftdi_ctx->readbuffer + ftdi_ctx->readbuffer_offset +
                      (packet_size - 2) * i,
                  ftdi_ctx->readbuffer + ftdi_ctx->readbuffer_offset +
                      packet_size * i,
                  packet_size - 2);
        if (chunk_remains > 2) {
          memmove(ftdi_ctx->readbuffer + ftdi_ctx->readbuffer_offset +
                      (packet_size - 2) * i,
                  ftdi_ctx->readbuffer + ftdi_ctx->readbuffer_offset +
                      packet_size * i,
                  chunk_remains - 2);
          actual_length -= 2 * num_of_chunks;
        } else
          actual_length -= 2 * (num_of_chunks - 1) + chunk_remains;
      }
    } else if (actual_length <= 2) {
      return offset;
    }
    if (actual_length > 0) {
      if (offset + actual_length <= size) {
        memcpy(buf + offset, ftdi_ctx->readbuffer + ftdi_ctx->readbuffer_offset,
               actual_length);
        offset += actual_length;
        if (offset == size) {
          return offset;
        }
      } else {
        int part_size = size - offset;
        memcpy(buf + offset, ftdi_ctx->readbuffer + ftdi_ctx->readbuffer_offset,
               part_size);
        ftdi_ctx->readbuffer_offset += part_size;
        ftdi_ctx->readbuffer_remaining = actual_length - part_size;
        offset += part_size;

        return offset;
      }
    }
  }
  return -3;
}
int ftdi_write_data(const unsigned char* buf, int size) {
  int offset = 0;
  int actual_length;
  int e = 0;
  while (offset < size) {
    int write_size = ftdi_ctx->writebuffer_chunksize;

    if (offset + write_size > size)
      write_size = size - offset;

    if ((e = bulk_transfer_func(ftdi_ctx->in_ep, (unsigned char*)buf + offset,
                                write_size, &actual_length,
                                ftdi_ctx->usb_write_timeout)) < 0) {
      return -1;
    }
    offset += actual_length;
  }

  return offset;
}
int ftdi_usb_open_dev() {
  struct usb_device_descriptor desc;
  struct usb_config_descriptor config0;
  int cfg0, cfg, r = 0;
  r = get_device_descriptor(&desc);
  if (r < 0) {
    return -1;
  }
  r = get_config_descriptor(0, &config0);
  if (r < 0) {
    return -2;
  }
  cfg0 = config0.bConfigurationValue;
  r = get_configuration(&cfg);
  r = set_configuration(cfg0);
  if (r < 0) {
    return -3;
  }
  r = claim_interface(ftdi_ctx->interface);
  if (r < 0) {
    return -4;
  }
  r = ftdi_usb_reset(ftdi_ctx);
  if (r != 0) {
    return -5;
  }
  if (desc.bcdDevice == 0x400 ||
      (desc.bcdDevice == 0x200 && desc.iSerialNumber == 0))
    ftdi_ctx->type = TYPE_BM;
  else if (desc.bcdDevice == 0x200)
    ftdi_ctx->type = TYPE_AM;
  else if (desc.bcdDevice == 0x500)
    ftdi_ctx->type = TYPE_2232C;
  else if (desc.bcdDevice == 0x600)
    ftdi_ctx->type = TYPE_R;
  else if (desc.bcdDevice == 0x700)
    ftdi_ctx->type = TYPE_2232H;
  else if (desc.bcdDevice == 0x800)
    ftdi_ctx->type = TYPE_4232H;
  else if (desc.bcdDevice == 0x900)
    ftdi_ctx->type = TYPE_232H;
  else if (desc.bcdDevice == 0x1000)
    ftdi_ctx->type = TYPE_230X;
  ftdi_ctx->max_packet_size = _ftdi_determine_max_packet_size();
  r = ftdi_set_baudrate(BAUD_RATE);
  if (r != 0) {
    return -6;
  }
  return 1;
}
int ftdi_open(int vid, int pid) {
  int ret;
  ret = usb_fetch(vid, pid);
  if (ret > 0) {
    ret = usb_open();
    if (ret) {
      ftdi_ctx->usb_ctx = usb_ctx;
      ftdi_ctx->usb_dev = handle;
      ret = ftdi_usb_open_dev();
      return ret;
    }
  }
  return 0;
}
void ftdi_close() {
  free(ftdi_ctx);
  free(ftdi_eep);
  free(ftdi_read_buf);
  usb_close();
}
int ftdi_get_state() {
  return usb_get_device_state();
}
