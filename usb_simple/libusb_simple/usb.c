/*
*	USB设备操作相关函数
*	修改自libusb，libusb开源地址：https://sourceforge.net/projects/libusb/files/
*	在源码基础上删除了所有未用到的功能和函数，增加了部分函数；根据需要，修改了所有动态内
*	存相关函数，所有动态内存都改为蓝牙模块初始化时分配，运行过程中不再动态分配内存；修改
*	了部分函数名；整合了多个源文件
*/
#include "types.h"
#include "usb.h"
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#define false 0
#define true 1
struct usb_context* usb_ctx;
struct usb_device* dest_device;
struct usb_device_handle* handle;
struct transfer* bulk_transfer;
struct transfer* ctrl_transfer;
struct usbi_pollfd* ipollfd_in;
struct usbi_pollfd* ipollfd_out;
#define bulk_buf_size 4096
#define ctrl_buf_size 1024
#define bulk_urb_num 256
unsigned char* bulk_buf;
struct usbfs_urb* bulk_urbs;
struct usbfs_urb* ctrl_urb;
unsigned char* ctrl_buf;
unsigned char* cfg_extra_bufs;
unsigned char* cfg_interface_bufs;
unsigned char* endpoint_bufs;
unsigned char* endpoint_extra_bufs;
unsigned char* interface_extra_bufs;
#define SYSFS_DEVICE_PATH "/sys/bus/usb/devices"
char dev_dir[128];
unsigned char dev_desc[18];
const char* usbfs_path= "/proc/bus/usb";
unsigned char cfg_desc[32];
// typedef unsigned char bool;
bool usb_mem_inited = false;
bool usb_fetched = false;
typedef uint16_t dev_id;
struct dev_set {
  dev_id deves[128];
  int size;
};
static inline void dev_set_add(struct dev_set* set, dev_id id) {
  if (set->size >= 128) {
    set->size = 0;
  }
  set->deves[set->size] = id;
  set->size++;
}
static inline void dev_set_del(struct dev_set* set) {
  if (set->size > 0) {
    set->size--;
  }
}
static inline bool dev_set_contain(struct dev_set* set, dev_id id) {
	int i;
  for (i = 0; i < set->size; i++) {
    if (set->deves[i] == id) {
      return true;
    }
  }
  return false;
}

struct dev_set scanned_devices = {deves : {0}, size : 0};
int claim_interface(int interface_number) {
  int r = 0;
  if (interface_number >= sizeof(handle->claimed_interfaces) * 8)
    return USB_ERROR_INVALID_PARAM;

  pthread_mutex_lock(&handle->lock);
  if (handle->claimed_interfaces & (1 << interface_number))
    goto out;

  r = usb_func.claim_interface(interface_number);
  if (r == 0)
    handle->claimed_interfaces |= 1 << interface_number;

out:
  pthread_mutex_unlock(&handle->lock);
  return r;
}
void lock_events() {
  pthread_mutex_lock(&usb_ctx->events_lock);
  usb_ctx->event_handler_active = 1;
}
void unlock_events() {
  usb_ctx->event_handler_active = 0;
  pthread_mutex_unlock(&usb_ctx->events_lock);
  pthread_mutex_lock(&usb_ctx->event_waiters_lock);
  pthread_cond_broadcast(&usb_ctx->event_waiters_cond);
  pthread_mutex_unlock(&usb_ctx->event_waiters_lock);
}
static void ctrl_transfer_cb() {
  int* completed = (int*)ctrl_transfer->usb_transfer.user_data;
  *completed = 1;
}
static int calculate_timeout(struct transfer* t) {
  int r;
  struct timespec current_time;
  unsigned int timeout = t->usb_transfer.timeout;

  if (!timeout)
    return 0;

  r = clock_gettime(CLOCK_REALTIME, &current_time);
  if (r < 0) {
    return r;
  }

  current_time.tv_sec += timeout / 1000;
  current_time.tv_nsec += (timeout % 1000) * 1000000;

  if (current_time.tv_nsec > 1000000000) {
    current_time.tv_nsec -= 1000000000;
    current_time.tv_sec++;
  }

  TIMESPEC_TO_TIMEVAL(&t->usbi_transfer.timeout, &current_time);
  return 0;
}

int submit_transfer(struct transfer* t) {
  int r;
  t->usbi_transfer.transferred = 0;
  t->usbi_transfer.flags = 0;
  r = calculate_timeout(t);
  if (r < 0) {
    return USB_ERROR_OTHER;
  }
  usb_ctx->flying_transfer = t;
  r = usb_func.submit_transfer(t);
  return r;
}
int _get_next_timeout(struct timeval* tv) {
  struct transfer* t = usb_ctx->flying_transfer;
  struct timespec cur_ts;
  struct timeval cur_tv;
  struct timeval* next_timeout;
  int r;
  int found = 0;
  if (!(t->usbi_transfer.flags & USBI_TRANSFER_TIMED_OUT)) {
    found = 1;
  }
  if (!found) {
    return 0;
  }

  next_timeout = &t->usbi_transfer.timeout;
  if (!timerisset(next_timeout)) {
    return 0;
  }

  r = clock_gettime(CLOCK_REALTIME, &cur_ts);
  if (r < 0) {
    return USB_ERROR_OTHER;
  }
  TIMESPEC_TO_TIMEVAL(&cur_tv, &cur_ts);
  if (timercmp(&cur_tv, next_timeout, >=)) {
    timerclear(tv);
  } else {
    timersub(next_timeout, &cur_tv, tv);
  }

  return 1;
}
static int get_next_timeout(struct timeval* tv, struct timeval* out) {
  struct timeval timeout;
  int r = _get_next_timeout(&timeout);
  if (r) {
    if (!timerisset(&timeout))
      return 1;
    if (timercmp(&timeout, tv, <))
      *out = timeout;
    else
      *out = *tv;
  } else {
    *out = *tv;
  }
  return 0;
}
int cancel_transfer(struct transfer* t) {
  int r;
  r = usb_func.cancel_transfer(t);
  return r;
}
static void handle_timeout(struct transfer* t) {
  int r;
  t->usbi_transfer.flags |= USBI_TRANSFER_TIMED_OUT;
  r = cancel_transfer(t);
}
static int handle_timeouts() {
  struct timespec systime_ts;
  struct timeval systime;
  struct timeval* cur_tv;
  struct transfer* t = usb_ctx->flying_transfer;
  int r = 0;
  if (!t) {
    return 0;
  }
  r = clock_gettime(CLOCK_REALTIME, &systime_ts);
  if (r < 0)
    goto out;

  TIMESPEC_TO_TIMEVAL(&systime, &systime_ts);
 cur_tv = &t->usbi_transfer.timeout;

  if (!timerisset(cur_tv)) {
    return r;
  }

  if ((cur_tv->tv_sec > systime.tv_sec) ||
      (cur_tv->tv_sec == systime.tv_sec && cur_tv->tv_usec > systime.tv_usec)) {
    return r;
  }
  handle_timeout(t);

out:
  return r;
}
int try_lock_events() {
  int r;
  pthread_mutex_lock(&usb_ctx->pollfd_modify_lock);
  r = usb_ctx->pollfd_modify;
  pthread_mutex_unlock(&usb_ctx->pollfd_modify_lock);
  if (r) {
    return 1;
  }

  r = pthread_mutex_trylock(&usb_ctx->events_lock);
  if (r)
    return 1;
  usb_ctx->event_handler_active = 1;
  return 0;
}
void unlock_event_waiters() {
  pthread_mutex_unlock(&usb_ctx->event_waiters_lock);
}
void lock_event_waiters() {
  pthread_mutex_lock(&usb_ctx->event_waiters_lock);
}
static int handle_events(struct timeval* tv) {
  int r;
  struct pollfd fds[2];
  int timeout_ms;
  fds[0].fd = ipollfd_in->pollfd.fd;
  fds[0].events = ipollfd_in->pollfd.events;
  fds[0].revents = 0;
  fds[1].fd = ipollfd_out->pollfd.fd;
  fds[1].events = ipollfd_out->pollfd.events;
  fds[1].revents = 0;
  timeout_ms = (tv->tv_sec * 1000) + (tv->tv_usec / 1000);
  if (tv->tv_usec % 1000)
    timeout_ms++;
  r = poll(fds, 2, timeout_ms);
  if (r == 0) {
    r = handle_timeouts();
    return r;
  } else if (r == -1 && errno == EINTR) {
    return USB_ERROR_INTERRUPTED;
  } else if (r < 0) {
    return USB_ERROR_IO;
  }
  if (fds[0].revents) {
    if (r == 1) {
      r = 0;
      goto handled;
    } else {
      fds[0].revents = 0;
    }
  }

  r = usb_func.handle_events(fds, 2);
handled:
  return r;
}
int event_handler_active() {
  int r;
  pthread_mutex_lock(&usb_ctx->pollfd_modify_lock);
  r = usb_ctx->pollfd_modify;
  pthread_mutex_unlock(&usb_ctx->pollfd_modify_lock);
  if (r) {
    return 1;
  }

  return usb_ctx->event_handler_active;
}
int wait_for_event(struct timeval* tv) {
  struct timespec timeout;
  int r;

  if (tv == NULL) {
    pthread_cond_wait(&usb_ctx->event_waiters_cond,
                      &usb_ctx->event_waiters_lock);
    return 0;
  }

  r = clock_gettime(CLOCK_REALTIME, &timeout);
  if (r < 0) {
    return USB_ERROR_OTHER;
  }

  timeout.tv_sec += tv->tv_sec;
  timeout.tv_nsec += tv->tv_usec * 1000;
  if (timeout.tv_nsec > 1000000000) {
    timeout.tv_nsec -= 1000000000;
    timeout.tv_sec++;
  }

  r = pthread_cond_timedwait(&usb_ctx->event_waiters_cond,
                             &usb_ctx->event_waiters_lock, &timeout);
  return (r == ETIMEDOUT);
}
int handle_events_timeout(struct timeval* tv) {
  int r;
  struct timeval poll_timeout;
  r = get_next_timeout(tv, &poll_timeout);
  if (r) {
    r = handle_timeouts();
    return r;
  }

retry:
  r = try_lock_events();
  if (r == 0) {
    r = handle_events(&poll_timeout);
    unlock_events();
    return r;
  }
  lock_event_waiters();

  if (!event_handler_active()) {
    unlock_event_waiters();
    goto retry;
  }
  r = wait_for_event(&poll_timeout);
  unlock_event_waiters();

  if (r < 0) {
    return r;
  } else if (r == 1) {
    r = handle_timeouts();
    return r;
  } else {
    return 0;
  }
}
int _handle_events() {
  struct timeval tv;
  int r;
  tv.tv_sec = 2;
  tv.tv_usec = 0;
  r = handle_events_timeout(&tv);
  return r;
}
static inline unsigned char* control_transfer_get_data() {
  return ctrl_transfer->usb_transfer.buffer + USB_CONTROL_SETUP_SIZE;
}
int control_transfer_func(uint8_t bmRequestType,
                          uint8_t bRequest,
                          uint16_t wValue,
                          uint16_t wIndex,
                          unsigned char* data,
                          uint16_t wLength,
                          unsigned int timeout) {
  int completed = 0;
  int r;

  fill_control_setup(ctrl_buf, bmRequestType, bRequest, wValue, wIndex,
                     wLength);
  if ((bmRequestType & USB_ENDPOINT_DIR_MASK) == USB_ENDPOINT_OUT)
    memcpy(ctrl_buf + USB_CONTROL_SETUP_SIZE, data, wLength);

  fill_control_transfer(ctrl_buf, ctrl_transfer_cb, &completed, timeout);
  ctrl_transfer->usb_transfer.flags = USB_TRANSFER_FREE_BUFFER;
  r = submit_transfer(ctrl_transfer);
  if (r < 0) {
    return r;
  }

  while (!completed) {
    r = _handle_events();
    if (r < 0) {
      cancel_transfer(ctrl_transfer);
      while (!completed) {
        if (_handle_events() < 0)
          break;
      }
      return r;
    }
  }

  if ((bmRequestType & USB_ENDPOINT_DIR_MASK) == USB_ENDPOINT_IN)
    memcpy(data, control_transfer_get_data(),
           ctrl_transfer->usb_transfer.actual_length);

  switch (ctrl_transfer->usb_transfer.status) {
    case USB_TRANSFER_COMPLETED:
      r = ctrl_transfer->usb_transfer.actual_length;
      break;
    case USB_TRANSFER_TIMED_OUT:
      r = USB_ERROR_TIMEOUT;
      break;
    case USB_TRANSFER_STALL:
      r = USB_ERROR_PIPE;
      break;
    case USB_TRANSFER_NO_DEVICE:
      r = USB_ERROR_NO_DEVICE;
      break;
    default:
      r = USB_ERROR_OTHER;
  }
  return r;
}
int set_configuration(int configuration) {
  int r = usb_func.set_configuration(configuration);
  return r;
}
int get_configuration(int* config) {
  int r;
  uint8_t tmp = 0;
  r = control_transfer_func(USB_ENDPOINT_IN, USB_REQUEST_GET_CONFIGURATION, 0,
                            0, &tmp, 1, 1000);
  if (r == 0) {
    r = USB_ERROR_IO;
  } else if (r == 1) {
    r = 0;
    *config = tmp;
  } else {
  }
  return r;
}
int parse_descriptor(unsigned char* source,
                     char* descriptor,
                     void* dest,
                     int host_endian) {
  unsigned char *sp = source, *dp = (unsigned char *)dest;
  uint16_t w;
  char* cp;

  for (cp = descriptor; *cp; cp++) {
    switch (*cp) {
      case 'b':
        *dp++ = *sp++;
        break;
      case 'w':
        dp += ((unsigned long)dp & 1);

        if (host_endian) {
          memcpy(dp, sp, 2);
        } else {
          w = (sp[1] << 8) | sp[0];
          *((uint16_t*)dp) = w;
        }
        sp += 2;
        dp += 2;
        break;
    }
  }

  return sp - source;
}
static int parse_endpoint(struct usb_endpoint_descriptor* endpoint,
                          unsigned char* buffer,
                          int size,
                          int host_endian) {
  struct usb_descriptor_header header;
  unsigned char* extra;
  unsigned char* begin;
  int parsed = 0;
  int len;
  int total = 0;
  parse_descriptor(buffer, "bb", &header, 0);
  if (header.bLength > size) {
    return -1;
  }
  if (header.bDescriptorType != USB_DT_ENDPOINT) {
    return parsed;
  }
  if (header.bLength >= ENDPOINT_AUDIO_DESC_LENGTH)
    parse_descriptor(buffer, "bbbbwbbb", endpoint, host_endian);
  else if (header.bLength >= ENDPOINT_DESC_LENGTH)
    parse_descriptor(buffer, "bbbbwb", endpoint, host_endian);

  buffer += header.bLength;
  size -= header.bLength;
  parsed += header.bLength;
  begin = buffer;
  while (size >= DESC_HEADER_LENGTH) {
    parse_descriptor(buffer, "bb", &header, 0);

    if (header.bLength < 2) {
      return -1;
    }
    if ((header.bDescriptorType == USB_DT_ENDPOINT) ||
        (header.bDescriptorType == USB_DT_INTERFACE) ||
        (header.bDescriptorType == USB_DT_CONFIG) ||
        (header.bDescriptorType == USB_DT_DEVICE))
      break;
    buffer += header.bLength;
    size -= header.bLength;
    parsed += header.bLength;
  }
  len = (int)(buffer - begin);
  if (!len) {
    endpoint->extra = NULL;
    endpoint->extra_length = 0;
    return parsed;
  }
  extra = (unsigned char*)(endpoint_extra_bufs + total);
  total += len;
  endpoint->extra = extra;
  memcpy(extra, begin, len);
  endpoint->extra_length = len;
  return parsed;
}

static int parse_interface(struct usb_interface* interface,
                           unsigned char* buffer,
                           int size,
                           int host_endian) {
  int i;
  int len;
  int r;
  int parsed = 0;
  int tmp;
  int total = 0;
  int total2 = 0;
  struct usb_descriptor_header header;
  struct usb_interface_descriptor* ifp;
  unsigned char* begin;

  interface->num_altsetting = 0;

  while (size >= INTERFACE_DESC_LENGTH) {
    struct usb_interface_descriptor* altsetting =
        (struct usb_interface_descriptor*)interface->altsetting;
    altsetting = (struct usb_interface_descriptor*)realloc(
        altsetting, sizeof(struct usb_interface_descriptor) *
                        (interface->num_altsetting + 1));
    if (!altsetting) {
      r = USB_ERROR_NO_MEM;
      goto err;
    }
    interface->altsetting = altsetting;

    ifp = altsetting + interface->num_altsetting;
    interface->num_altsetting++;
    parse_descriptor(buffer, "bbbbbbbbb", ifp, 0);
    ifp->extra = NULL;
    ifp->extra_length = 0;
    ifp->endpoint = NULL;

    /* Skip over the interface */
    buffer += ifp->bLength;
    parsed += ifp->bLength;
    size -= ifp->bLength;

    begin = buffer;

    /* Skip over any interface, class or vendor descriptors */
    while (size >= DESC_HEADER_LENGTH) {
      parse_descriptor(buffer, "bb", &header, 0);
      if (header.bLength < 2) {
        r = USB_ERROR_IO;
        goto err;
      }

      /* If we find another "proper" descriptor then we're done */
      if ((header.bDescriptorType == USB_DT_INTERFACE) ||
          (header.bDescriptorType == USB_DT_ENDPOINT) ||
          (header.bDescriptorType == USB_DT_CONFIG) ||
          (header.bDescriptorType == USB_DT_DEVICE))
        break;

      buffer += header.bLength;
      parsed += header.bLength;
      size -= header.bLength;
    }
    len = (int)(buffer - begin);
    if (len) {
      ifp->extra = (const unsigned char*)(interface_extra_bufs + total2);
      total2 += len;
      memcpy((unsigned char*)ifp->extra, begin, len);
      ifp->extra_length = len;
    }
    parse_descriptor(buffer, "bb", &header, 0);
    if ((size >= DESC_HEADER_LENGTH) &&
        ((header.bDescriptorType == USB_DT_CONFIG) ||
         (header.bDescriptorType == USB_DT_DEVICE))) {
      return parsed;
    }

    if (ifp->bNumEndpoints > USB_MAXENDPOINTS) {
      r = USB_ERROR_IO;
      goto err;
    }

    if (ifp->bNumEndpoints > 0) {
      struct usb_endpoint_descriptor* endpoint;
      tmp = ifp->bNumEndpoints * sizeof(struct usb_endpoint_descriptor);
      endpoint = (struct usb_endpoint_descriptor*)memset(endpoint_bufs + total,
                                                         0, tmp);
      total += tmp;
      ifp->endpoint = endpoint;
      for (i = 0; i < ifp->bNumEndpoints; i++) {
        parse_descriptor(buffer, "bb", &header, 0);

        if (header.bLength > size) {
          r = USB_ERROR_IO;
          goto err;
        }

        r = parse_endpoint(endpoint + i, buffer, size, host_endian);
        if (r < 0)
          goto err;

        buffer += r;
        parsed += r;
        size -= r;
      }
    }
    ifp = (struct usb_interface_descriptor*)buffer;
    if (size < USB_DT_INTERFACE_SIZE ||
        ifp->bDescriptorType != USB_DT_INTERFACE || !ifp->bAlternateSetting) {
      return parsed;
    }
  }
  return parsed;
err:
  return r;
}

static int parse_configuration(struct usb_config_descriptor* config,
                               unsigned char* buffer,
                               int host_endian) {
  int i;
  int r;
  int size;
  int tmp;
  int total = 0;
  struct usb_descriptor_header header;
  struct usb_interface* interface;

  parse_descriptor(buffer, "bbwbbbbb", config, host_endian);
  size = config->wTotalLength;

  if (config->bNumInterfaces > USB_MAXINTERFACES) {
    return USB_ERROR_IO;
  }

  tmp = config->bNumInterfaces * sizeof(struct usb_interface);
  interface = (struct usb_interface*)memset(cfg_interface_bufs, 0, tmp);
  config->interface = interface;
  buffer += config->bLength;
  size -= config->bLength;

  config->extra = NULL;
  config->extra_length = 0;

  for (i = 0; i < config->bNumInterfaces; i++) {
    int len;
    unsigned char* begin;
    begin = buffer;
    while (size >= DESC_HEADER_LENGTH) {
      parse_descriptor(buffer, "bb", &header, 0);

      if ((header.bLength > size) || (header.bLength < DESC_HEADER_LENGTH)) {
        r = USB_ERROR_IO;
        goto err;
      }
      if ((header.bDescriptorType == USB_DT_ENDPOINT) ||
          (header.bDescriptorType == USB_DT_INTERFACE) ||
          (header.bDescriptorType == USB_DT_CONFIG) ||
          (header.bDescriptorType == USB_DT_DEVICE))
        break;
      buffer += header.bLength;
      size -= header.bLength;
    }
    len = (int)(buffer - begin);
    if (len) {
      if (!config->extra_length) {
        config->extra = (const unsigned char*)&cfg_extra_bufs[total];
        total += len;
        if (!config->extra) {
          r = USB_ERROR_NO_MEM;
          goto err;
        }
        memcpy((unsigned char*)config->extra, begin, len);
        config->extra_length = len;
      }
    }

    r = parse_interface(interface + i, buffer, size, host_endian);
    if (r < 0)
      goto err;

    buffer += r;
    size -= r;
  }
  return size;

err:
  return r;
}
int get_config_descriptor(uint8_t config_index,
                          struct usb_config_descriptor* config) {
  struct usb_config_descriptor _config;
  unsigned char tmp[8];
  unsigned char buf[256];
  int host_endian = 0;
  int r;
  if (config_index >= dest_device->num_configurations)
    return USB_ERROR_NOT_FOUND;

  r = usb_func.get_config_descriptor(config_index, tmp, sizeof(tmp),
                                     &host_endian);
  if (r < 0)
    goto err;

  parse_descriptor(tmp, "bbw", &_config, host_endian);
  r = usb_func.get_config_descriptor(config_index, buf, _config.wTotalLength,
                                     &host_endian);
  if (r < 0)
    goto err;

  r = parse_configuration(&_config, buf, host_endian);
  if (r < 0) {
    goto err;
  } else if (r > 0) {
  }

  *config = _config;
  return 0;
err:
  return r;
}
int get_config_index_by_value(uint8_t bConfigurationValue, int* idx) {
  int i;
  for (i = 0; i < dest_device->num_configurations; i++) {
    unsigned char tmp[6];
    int host_endian;
    int r = usb_func.get_config_descriptor(i, tmp, sizeof(tmp), &host_endian);
    if (r < 0) {
      return r;
    }
    if (tmp[5] == bConfigurationValue) {
      *idx = i;
      return 0;
    }
  }

  *idx = -1;
  return 0;
}
int get_config_descriptor_by_value(uint8_t bConfigurationValue,
                                   struct usb_config_descriptor** config) {
  int idx;
  int r = get_config_index_by_value(bConfigurationValue, &idx);
  if (r < 0) {
    return r;
  } else if (idx == -1) {
    return USB_ERROR_NOT_FOUND;
  } else {
    r = get_config_descriptor(idx, *config);
    return r;
  }
}
int get_string_descriptor_ascii(uint8_t desc_index,
                                unsigned char* data,
                                int length) {
  unsigned char tbuf[255];
  int r, langid, si, di;
  r = get_string_descriptor(0, 0, tbuf, sizeof(tbuf));
  if (r < 0) {
    return r;
  }
  if (r < 4) {
    return USB_ERROR_IO;
  }

  langid = tbuf[2] | (tbuf[3] << 8);

  r = get_string_descriptor(desc_index, langid, tbuf, sizeof(tbuf));
  if (r < 0) {
    return r;
  }

  if (tbuf[1] != USB_DT_STRING) {
    return USB_ERROR_IO;
  }

  if (tbuf[0] > r) {
    return USB_ERROR_IO;
  }

  for (di = 0, si = 2; si < tbuf[0]; si += 2) {
    if (di >= (length - 1))
      break;

    if (tbuf[si + 1])
      data[di++] = '?';
    else
      data[di++] = tbuf[si];
  }

  data[di] = 0;
  return di;
}

int get_device_descriptor(struct usb_device_descriptor* desc) {
  unsigned char raw_desc[18];
  int host_endian = 0;
  int r;
  r = usb_func.get_device_descriptor(raw_desc, &host_endian);
  if (r < 0) {
    return r;
  }
  memcpy((unsigned char*)desc, raw_desc, sizeof(raw_desc));
  if (host_endian) {
    desc->bcdUSB = usb_cpu_to_le16(desc->bcdUSB);
    desc->idVendor = usb_cpu_to_le16(desc->idVendor);
    desc->idProduct = usb_cpu_to_le16(desc->idProduct);
    desc->bcdDevice = usb_cpu_to_le16(desc->bcdDevice);
  }
  return 0;
}
static int check_usb_vfs(const char* dirname) {
  DIR* dir;
  struct dirent* entry;
  int found = 0;

  dir = opendir(dirname);
  if (!dir)
    return 0;

  while ((entry = readdir(dir)) != NULL) {
    if (entry->d_name[0] == '.')
      continue;

    found = 1;
    break;
  }

  closedir(dir);
  return found;
}
static const char* find_usbfs_path(void) {
  const char* path = "/dev/bus/usb";
  const char* ret = NULL;

  if (check_usb_vfs(path)) {
    ret = path;
  } else {
    path = "/proc/bus/usb";
    if (check_usb_vfs(path))
      ret = path;
  }
  return ret;
}
int io_init() {
  int r;
  pthread_mutex_init(&usb_ctx->pollfd_modify_lock, NULL);
  pthread_mutex_init(&usb_ctx->events_lock, NULL);
  pthread_mutex_init(&usb_ctx->event_waiters_lock, NULL);
  pthread_cond_init(&usb_ctx->event_waiters_cond, NULL);
  r = pipe(usb_ctx->ctrl_pipe);
  if (r < 0)
    return USB_ERROR_OTHER;
  ipollfd_in->pollfd.fd = usb_ctx->ctrl_pipe[0];
  ipollfd_in->pollfd.events = POLLIN;
  return 0;
}
int usb_mem_init(void) {
  int r = 0;
  usb_ctx = (struct usb_context*)malloc(sizeof(struct usb_context));
  if (!usb_ctx) {
    r = -1;
    goto out;
  }
  dest_device = (struct usb_device*)malloc(sizeof(struct usb_device));
  if (!dest_device) {
    r = -2;
    goto i1;
  }
  handle = (struct usb_device_handle*)malloc(sizeof(struct usb_device_handle));
  if (!handle) {
    r = -3;
    goto i2;
  }
  bulk_transfer = (struct transfer*)malloc(sizeof(struct transfer));
  if (!bulk_transfer) {
    r = -4;
    goto i3;
  }
  ctrl_transfer = (struct transfer*)malloc(sizeof(struct transfer));
  if (!ctrl_transfer) {
    r = -5;
    goto i4;
  }
  bulk_buf = (unsigned char*)malloc(bulk_buf_size);
  if (!bulk_buf) {
    r = -6;
    goto i5;
  }
  bulk_urbs =
      (struct usbfs_urb*)malloc(sizeof(struct usbfs_urb) * bulk_urb_num);
  if (!bulk_urbs) {
    r = -7;
    goto i6;
  }
  ctrl_urb = (struct usbfs_urb*)malloc(sizeof(struct usbfs_urb));
  if (!ctrl_urb) {
    r = -8;
    goto i7;
  }
  ctrl_buf = (unsigned char*)malloc(ctrl_buf_size);
  if (!ctrl_buf) {
    r = -9;
    goto i8;
  }
  cfg_extra_bufs = (unsigned char*)malloc(4096);
  if (!cfg_extra_bufs) {
    r = -10;
    goto i9;
  }
  cfg_interface_bufs = (unsigned char*)malloc(4096);
  if (!cfg_interface_bufs) {
    r = -11;
    goto i10;
  }
  endpoint_bufs = (unsigned char*)malloc(4096);
  if (!endpoint_bufs) {
    r = -12;
    goto i11;
  }
  endpoint_extra_bufs = (unsigned char*)malloc(4096);
  if (!endpoint_extra_bufs) {
    r = -13;
    goto i12;
  }
  interface_extra_bufs = (unsigned char*)malloc(4096);
  if (!interface_extra_bufs) {
    r = -14;
    goto i13;
  }
  ipollfd_in = (struct usbi_pollfd*)malloc(sizeof(struct usbi_pollfd));
  if (!ipollfd_in) {
    r = -15;
    goto i14;
  }
  ipollfd_out = (struct usbi_pollfd*)malloc(sizeof(struct usbi_pollfd));
  if (!ipollfd_out) {
    r = -16;
    goto i15;
  }
  usb_mem_inited = true;
  return r;
  FREE(ipollfd_out);
i15:
  FREE(ipollfd_in);
i14:
  FREE(interface_extra_bufs);
i13:
  FREE(endpoint_extra_bufs);
i12:
  FREE(endpoint_bufs);
i11:
  FREE(cfg_interface_bufs);
i10:
  FREE(cfg_extra_bufs);
i9:
  FREE(ctrl_buf);
i8:
  FREE(ctrl_urb);
i7:
  FREE(bulk_urbs);
i6:
  FREE(bulk_buf);
i5:
  FREE(ctrl_transfer);
i4:
  FREE(bulk_transfer);
i3:
  FREE(handle);
i2:
  FREE(dest_device);
i1:
  FREE(usb_ctx);
out:
  return r;
}
static int get_active_config(int fd) {
  unsigned char active_config = 0;
  int r;

  struct usbfs_ctrltransfer ctrl = {
    bmRequestType : USB_ENDPOINT_IN,
    bRequest : USB_REQUEST_GET_CONFIGURATION,
    wValue : 0,
    wIndex : 0,
    wLength : 1,
    timeout : 1000,
    data : &active_config
  };

  r = ioctl(fd, IOCTL_USBFS_CONTROL, &ctrl);
  if (r < 0) {
    if (errno == ENODEV)
      return USB_ERROR_NO_DEVICE;
    return USB_ERROR_IO;
  }

  return active_config;
}
static int seek_to_next_config(int fd) {
  struct usb_config_descriptor config;
  unsigned char tmp[6];
  off_t off;
  int r;
  r = read(fd, tmp, sizeof(tmp));
  if (r < 0) {
    return USB_ERROR_IO;
  } else if (r < sizeof(tmp)) {
    return USB_ERROR_IO;
  }
  parse_descriptor(tmp, "bbwbb", &config, 1);
  off = lseek(fd, config.wTotalLength - sizeof(tmp), SEEK_CUR);
  if (off < 0) {
    return USB_ERROR_IO;
  }
  return 0;
}
static int _get_config_descriptor(int fd,
                                  uint8_t config_index,
                                  unsigned char* buffer,
                                  size_t len) {
  off_t off;
  ssize_t r;

  off = lseek(fd, 18, 0);
  if (off < 0) {
    return USB_ERROR_IO;
  }
  while (config_index > 0) {
    r = seek_to_next_config(fd);
    if (r < 0) {
      return r;
    }
    config_index--;
  }
  r = read(fd, buffer, len);
  if (r < 0) {
    return USB_ERROR_IO;
  } else if (r < len) {
    return USB_ERROR_IO;
  }
  return 0;
}
static int cache_active_config(int fd, int active_config) {
  struct usb_config_descriptor config;
  unsigned char tmp[8];
  int idx;
  int r;

  if (active_config == -1) {
    idx = 0;
  } else {
    r = get_config_index_by_value(active_config, &idx);
    if (r < 0) {
      return -1;
    }
    if (idx == -1) {
      return -2;
    }
  }

  r = _get_config_descriptor(fd, idx, tmp, sizeof(tmp));
  if (r < 0) {
    return r;
  }

  parse_descriptor(tmp, "bbw", &config, 1);
  r = _get_config_descriptor(fd, idx, cfg_desc, config.wTotalLength);
  if (r < 0) {
    return -3;
  }
  return 0;
}

int usbi_sanitize_device(struct usb_device* dev) {
  int r;
  unsigned char raw_desc[18];
  uint8_t num_configurations;
  int host_endian;

  r = usb_func.get_device_descriptor(raw_desc, &host_endian);
  if (r < 0) {
    return r;
  }

  num_configurations = raw_desc[17];
  if (num_configurations > 8) {
    return USB_ERROR_IO;
  } else if (num_configurations < 1) {
    return USB_ERROR_IO;
  }
  dev->num_configurations = num_configurations;
  return 0;
}
static void get_usbfs_path(int busnum, int devaddr, char* path) {
  snprintf(path, PATH_MAX, "%s/%03d/%03d", usbfs_path, busnum, devaddr);
}
static int enumerate_device(uint8_t busnum, uint8_t devaddr, int vid, int pid) {
  int r = 0;
  char path[PATH_MAX];
  struct usb_device_descriptor* desc;
  int fd, active_config, device_configured = 1;
  get_usbfs_path(busnum, devaddr, path);
  fd = open(path, O_RDWR);
  if (fd <= 0) {
    return 0;
  }
  active_config = get_active_config(fd);
  if (active_config < 0) {
    close(fd);
    return 0;
  } else if (active_config == 0) {
    device_configured = 0;
  }
  r = read(fd, dev_desc, 18);
  desc = (struct usb_device_descriptor*)dev_desc;
  if (r < 18) {
    close(fd);
    return 0;
  }
  if (desc->idVendor == vid && desc->idProduct == pid) {
    dest_device->dev_priv.dev_descriptor = (unsigned char*)&dev_desc;
    dest_device->num_configurations = desc->bNumConfigurations;
    dest_device->bus_number = busnum;
    dest_device->device_address = devaddr;
    strcpy(dest_device->usbfs_dev_path, path);
    if (device_configured) {
      r = cache_active_config(fd, active_config);
      if (r < 0) {
        close(fd);
        return 0;
      }
      dest_device->dev_priv.config_descriptor = cfg_desc;
    }
    close(fd);
    return 1;
  }
  close(fd);
  return 0;
}
static int scan_busdir(uint8_t busnum, int vid, int pid) {
  DIR* dir;
  char dirpath[PATH_MAX];
  struct dirent* entry;
  int r = 0;
  snprintf(dirpath, PATH_MAX, "%s/%03d", usbfs_path, busnum);
  dir = opendir(dirpath);
  if (!dir) {
    return 0;
  }
  while ((entry = readdir(dir))) {
    int devaddr;
    if (entry->d_name[0] == '.')
      continue;
    devaddr = atoi(entry->d_name);
    if (devaddr == 0 ||
        dev_set_contain(&scanned_devices, busnum << 8 | devaddr)) {
      continue;
    }
    r = enumerate_device(busnum, devaddr, vid, pid);
    dev_set_add(&scanned_devices, busnum << 8 | devaddr);
    if (r) {
      return 1;
    }
  }
out:
  closedir(dir);
  return 0;
}
void usb_mem_reset() {
  memset(usb_ctx, 0, sizeof(struct usb_context));
  memset(dest_device, 0, sizeof(struct usb_device));
  memset(handle, 0, sizeof(struct usb_device_handle));
  memset(bulk_transfer, 0, sizeof(struct transfer));
  memset(ctrl_transfer, 0, sizeof(struct transfer));
  memset(bulk_buf, 0, bulk_buf_size);
  memset(bulk_urbs, 0, sizeof(struct usbfs_urb) * bulk_urb_num);
  memset(ctrl_urb, 0, sizeof(struct usbfs_urb));
  memset(ctrl_buf, 0, ctrl_buf_size);
  memset(cfg_extra_bufs, 0, 4096);
  memset(cfg_interface_bufs, 0, 4096);
  memset(endpoint_bufs, 0, 4096);
  memset(endpoint_extra_bufs, 0, 4096);
  memset(interface_extra_bufs, 0, 4096);
  memset(ipollfd_in, 0, sizeof(struct usbi_pollfd));
  memset(ipollfd_out, 0, sizeof(struct usbi_pollfd));
}

int usb_fetch(int vid, int pid) {
  if (usb_mem_inited) {
	  DIR* devs_dir;
	  struct dirent* entry;
	  int busnum = 0, r;
	  bool found = false;
    usb_fetched = false;
	devs_dir = opendir(usbfs_path);
    usb_mem_reset();
    dev_set_del(&scanned_devices);
    r = io_init();
    if (r < 0) {
      return -1;
    }
    while ((entry = readdir(devs_dir))) {
      if (entry->d_name[0] == '.')
        continue;
      busnum = atoi(entry->d_name);
      r = scan_busdir(busnum, vid, pid);
      if (r) {
        found = true;
        break;
      }
    }
    closedir(devs_dir);
    if (!found) {
      return -2;
    }
    usb_fetched = true;
    return 1;
  }
  return -3;
}
int usb_get_device_state() {
  if (usb_fetched && strlen(dest_device->usbfs_dev_path) > 0) {
    if (access(dest_device->usbfs_dev_path, 0) == 0) {
      return USB_STAT_PLUGED;
    } else {
      usb_fetched = false;
      return USB_STAT_UNPLUGED;
    }
  }
  return USB_STAT_UNPLUGED;
}

int usb_open() {
  unsigned char dummy = 1;
  int r;
  r = pthread_mutex_init(&handle->lock, NULL);
  if (r) {
    return -1;
  }

  handle->claimed_interfaces = 0;
  r = usb_func.open();
  if (r < 0) {
    return -2;
  }
  pthread_mutex_lock(&usb_ctx->pollfd_modify_lock);
  usb_ctx->pollfd_modify++;
  pthread_mutex_unlock(&usb_ctx->pollfd_modify_lock);
  r = write(usb_ctx->ctrl_pipe[1], &dummy, sizeof(dummy));
  if (r <= 0) {
    pthread_mutex_lock(&usb_ctx->pollfd_modify_lock);
    usb_ctx->pollfd_modify--;
    pthread_mutex_unlock(&usb_ctx->pollfd_modify_lock);
    return 1;
  }
  lock_events();
  r = read(usb_ctx->ctrl_pipe[0], &dummy, sizeof(dummy));
  pthread_mutex_lock(&usb_ctx->pollfd_modify_lock);
  usb_ctx->pollfd_modify--;
  pthread_mutex_unlock(&usb_ctx->pollfd_modify_lock);
  unlock_events();
  return 1;
}
static int op_open() {
  struct linux_device_handle_priv* hpriv = &handle->priv;
  char filename[PATH_MAX];
  snprintf(filename, PATH_MAX, "%s/%03d/%03d", usbfs_path,
           dest_device->bus_number, dest_device->device_address);
  hpriv->fd = open(filename, O_RDWR);
  if (hpriv->fd < 0) {
    if (errno == EACCES) {
      return USB_ERROR_ACCESS;
    } else if (errno == ENOENT) {
      return USB_ERROR_NO_DEVICE;
    } else {
      return USB_ERROR_IO;
    }
  }
  ipollfd_out->pollfd.fd = hpriv->fd;
  ipollfd_out->pollfd.events = POLLOUT;
  return 0;
}
static int _get_device_descriptor(unsigned char* buffer) {
  struct linux_device_priv* priv = &dest_device->dev_priv;
  memcpy(buffer, priv->dev_descriptor, 18);
  return 0;
}
static int op_get_device_descriptor(unsigned char* buffer, int* host_endian) {
	int r;
	*host_endian = 1;
 r = _get_device_descriptor(buffer);
  return r;
}

static int op_get_config_descriptor(uint8_t config_index,
                                    unsigned char* buffer,
                                    size_t len,
                                    int* host_endian) {
  char filename[PATH_MAX];
  int fd;
  int r;
  snprintf(filename, PATH_MAX, "%s/%03d/%03d", usbfs_path,
           dest_device->bus_number, dest_device->device_address);
  fd = open(filename, O_RDONLY);
  if (fd < 0) {
    return -1;
  }

  r = _get_config_descriptor(fd, config_index, buffer, len);
  close(fd);
  *host_endian = 1;
  return r;
}

static int op_set_configuration(int config) {
  int fd = handle->priv.fd;
  int r = ioctl(fd, IOCTL_USBFS_SETCONFIG, &config);
  if (r) {
    if (errno == EINVAL) {
      return USB_ERROR_NOT_FOUND;
    } else if (errno == EBUSY) {
      return USB_ERROR_BUSY;
    } else if (errno == ENODEV) {
      return USB_ERROR_NO_DEVICE;
    }
    return USB_ERROR_OTHER;
  }

  if (config == -1) {
  } else {
    r = cache_active_config(fd, config);
  }
  return 0;
}
void usb_close() {
  if (handle&&handle->priv.fd)close(handle->priv.fd);
  FREE(ipollfd_out);
  FREE(ipollfd_in);
  FREE(interface_extra_bufs);
  FREE(endpoint_extra_bufs);
  FREE(endpoint_bufs);
  FREE(cfg_interface_bufs);
  FREE(cfg_extra_bufs);
  FREE(ctrl_buf);
  FREE(ctrl_urb);
  FREE(bulk_urbs);
  FREE(bulk_buf);
  FREE(ctrl_transfer);
  FREE(bulk_transfer);
  FREE(handle);
  FREE(dest_device);
  FREE(usb_ctx);
}
static int op_claim_interface(int iface) {
  int fd = handle->priv.fd;
  int r = ioctl(fd, IOCTL_USBFS_CLAIMINTF, &iface);
  if (r) {
    if (errno == ENOENT) {
      return USB_ERROR_NOT_FOUND;
    } else if (errno == EBUSY) {
      return USB_ERROR_BUSY;
    } else if (errno == ENODEV) {
      return USB_ERROR_NO_DEVICE;
    }
    return USB_ERROR_OTHER;
  }
  return 0;
}
static void bulk_transfer_cb() {
  int* completed = (int*)bulk_transfer->usb_transfer.user_data;
  *completed = 1;
}
static int do_sync_bulk_transfer(unsigned char endpoint,
                                 unsigned char* buffer,
                                 int length,
                                 int* transferred,
                                 unsigned int timeout,
                                 unsigned char type) {
  int completed = 0;
  int r;
  fill_bulk_transfer(endpoint, buffer, length, bulk_transfer_cb, &completed,
                     timeout);
  bulk_transfer->usb_transfer.type = type;

  r = submit_transfer(bulk_transfer);
  if (r < 0) {
    return r;
  }

  while (!completed) {
    r = _handle_events();
    if (r < 0) {
      cancel_transfer(bulk_transfer);
      while (!completed)
        if (_handle_events() < 0)
          break;
      return r;
    }
  }

  *transferred = bulk_transfer->usb_transfer.actual_length;
  switch (bulk_transfer->usb_transfer.status) {
    case USB_TRANSFER_COMPLETED:
      r = 0;
      break;
    case USB_TRANSFER_TIMED_OUT:
      r = USB_ERROR_TIMEOUT;
      break;
    case USB_TRANSFER_STALL:
      r = USB_ERROR_PIPE;
      break;
    case USB_TRANSFER_OVERFLOW:
      r = USB_ERROR_OVERFLOW;
      break;
    case USB_TRANSFER_NO_DEVICE:
      r = USB_ERROR_NO_DEVICE;
      break;
    default:
      r = USB_ERROR_OTHER;
  }
  return r;
}
int bulk_transfer_func(unsigned char endpoint,
                       unsigned char* data,
                       int length,
                       int* transferred,
                       unsigned int timeout) {
  int r = do_sync_bulk_transfer(endpoint, data, length, transferred, timeout,
                                USB_TRANSFER_TYPE_BULK);
  return r;
}
static int submit_control_transfer(struct transfer* t) {
  struct linux_transfer_priv* tpriv = &t->usb_transfer.transfer_priv;
  struct usb_transfer* transfer = &t->usb_transfer;
  struct linux_device_handle_priv* dpriv = &handle->priv;
  int r;

  if (transfer->length - USB_CONTROL_SETUP_SIZE > MAX_CTRL_BUFFER_LENGTH) {
    return USB_ERROR_INVALID_PARAM;
  }

  memset(ctrl_urb, 0, sizeof(struct usbfs_urb));
  tpriv->urbS.urbs = ctrl_urb;
  tpriv->reap_action = NORMAL;

  ctrl_urb->usercontext = t;
  ctrl_urb->type = USBFS_URB_TYPE_CONTROL;
  ctrl_urb->endpoint = transfer->endpoint;
  ctrl_urb->buffer = transfer->buffer;
  ctrl_urb->buffer_length = transfer->length;

  r = ioctl(dpriv->fd, IOCTL_USBFS_SUBMITURB, ctrl_urb);
  if (r < 0) {
    if (errno == ENODEV) {
      return USB_ERROR_NO_DEVICE;
    }
    return USB_ERROR_IO;
  }
  return 0;
}
static int submit_bulk_transfer(struct transfer* t, unsigned char urb_type) {
  struct usb_transfer* transfer = &t->usb_transfer;
  struct linux_transfer_priv* tpriv = &t->usb_transfer.transfer_priv;
  struct linux_device_handle_priv* dpriv = &handle->priv;
  int r;
  int i;
  size_t alloc_size;
  int num_urbs = transfer->length / MAX_BULK_BUFFER_LENGTH;
  int last_urb_partial = 0;
  if ((transfer->length % MAX_BULK_BUFFER_LENGTH) > 0) {
    last_urb_partial = 1;
    num_urbs++;
  }
  alloc_size = num_urbs * sizeof(struct usbfs_urb);
  memset(bulk_urbs, 0, alloc_size);
  tpriv->urbS.urbs = bulk_urbs;
  tpriv->num_urbs = num_urbs;
  tpriv->awaiting_discard = 0;
  tpriv->awaiting_reap = 0;
  tpriv->reap_action = NORMAL;

  for (i = 0; i < num_urbs; i++) {
    struct usbfs_urb* urb = &bulk_urbs[i];
    urb->usercontext = t;
    urb->type = urb_type;
    urb->endpoint = transfer->endpoint;
    urb->buffer = transfer->buffer + (i * MAX_BULK_BUFFER_LENGTH);
    if (i == num_urbs - 1 && last_urb_partial)
      urb->buffer_length = transfer->length % MAX_BULK_BUFFER_LENGTH;
    else
      urb->buffer_length = MAX_BULK_BUFFER_LENGTH;

    r = ioctl(dpriv->fd, IOCTL_USBFS_SUBMITURB, urb);
    if (r < 0) {
      int j;
      if (errno == ENODEV) {
        r = USB_ERROR_NO_DEVICE;
      } else {
        perror("ioctl");
        r = USB_ERROR_IO;
      }
      if (i == 0) {
        return r;
      }
      tpriv->reap_action = SUBMIT_FAILED;
      for (j = 0; j < i; j++) {
        int tmp = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, &bulk_urbs[j]);
        if (tmp == 0)
          tpriv->awaiting_discard++;
        else if (errno == EINVAL)
          tpriv->awaiting_reap++;
        else
          ;
      }
      return 0;
    }
  }
  return 0;
}
static int op_submit_transfer(struct transfer* t) {
  struct usb_transfer* transfer = &t->usb_transfer;
  int r = 0;
  switch (transfer->type) {
    case USB_TRANSFER_TYPE_CONTROL:
      r = submit_control_transfer(t);
      return r;
    case USB_TRANSFER_TYPE_BULK:
      r = submit_bulk_transfer(t, USBFS_URB_TYPE_BULK);
      return r;
    case USB_TRANSFER_TYPE_INTERRUPT:
      r = submit_bulk_transfer(t, USBFS_URB_TYPE_INTERRUPT);
      return r;
    default:
      return USB_ERROR_INVALID_PARAM;
  }
}
static int cancel_control_transfer(struct transfer* t) {
  struct linux_transfer_priv* tpriv = &t->usb_transfer.transfer_priv;
  struct linux_device_handle_priv* dpriv = &handle->priv;
  int r;

  tpriv->reap_action = CANCELLED;
  r = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, tpriv->urbS.urbs);
  if (r) {
    if (errno == EINVAL) {
      return 0;
    } else {
      return USB_ERROR_OTHER;
    }
  }
  return 0;
}
static void cancel_bulk_transfer(struct transfer* t) {
  struct linux_transfer_priv* tpriv = &t->usb_transfer.transfer_priv;
  struct usb_transfer* transfer = &t->usb_transfer;
  struct linux_device_handle_priv* dpriv = &handle->priv;
  int i;

  tpriv->reap_action = CANCELLED;
  tpriv->awaiting_reap = 0;
  tpriv->awaiting_discard = 0;
  for (i = 0; i < tpriv->num_urbs; i++) {
    int tmp = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, &tpriv->urbS.urbs[i]);
    if (tmp == 0)
      tpriv->awaiting_discard++;
    else if (errno == EINVAL)
      tpriv->awaiting_reap++;
    else
      ;
  }
}

static int op_cancel_transfer(struct transfer* t) {
  struct usb_transfer* transfer = &t->usb_transfer;
  int r = 0;
  switch (transfer->type) {
    case USB_TRANSFER_TYPE_CONTROL:
      r = cancel_control_transfer(t);
      return r;
    case USB_TRANSFER_TYPE_BULK:
    case USB_TRANSFER_TYPE_INTERRUPT:
      cancel_bulk_transfer(t);
      return 0;
    default:
      return USB_ERROR_INVALID_PARAM;
  }
}
void usbi_handle_transfer_completion(struct transfer* t,
                                     enum usb_transfer_status status) {
  struct usb_transfer* transfer = &t->usb_transfer;
  uint8_t flags;

  usb_ctx->flying_transfer = NULL;

  if (status == USB_TRANSFER_COMPLETED &&
      transfer->flags & USB_TRANSFER_SHORT_NOT_OK) {
    int rqlen = transfer->length;
    if (transfer->type == USB_TRANSFER_TYPE_CONTROL)
      rqlen -= USB_CONTROL_SETUP_SIZE;
    if (rqlen != t->usbi_transfer.transferred) {
      status = USB_TRANSFER_ERROR;
    }
  }

  flags = transfer->flags;
  transfer->status = status;
  transfer->actual_length = t->usbi_transfer.transferred;
  if (transfer->callback)
    transfer->callback();
  pthread_mutex_lock(&usb_ctx->event_waiters_lock);
  pthread_cond_broadcast(&usb_ctx->event_waiters_cond);
  pthread_mutex_unlock(&usb_ctx->event_waiters_lock);
}
void usbi_handle_transfer_cancellation(struct transfer* t) {
  if (t->usbi_transfer.flags & USBI_TRANSFER_TIMED_OUT) {
    usbi_handle_transfer_completion(t, USB_TRANSFER_TIMED_OUT);
    return;
  }
  usbi_handle_transfer_completion(t, USB_TRANSFER_CANCELLED);
}

static int handle_control_completion(struct transfer* t,
                                     struct usbfs_urb* urb) {
  struct linux_transfer_priv* tpriv = &t->usb_transfer.transfer_priv;
  int status;

  if (urb->status == 0)
    t->usbi_transfer.transferred += urb->actual_length;

  if (tpriv->reap_action == CANCELLED) {
    if (urb->status != 0 && urb->status != -ENOENT)
      ;
    usbi_handle_transfer_cancellation(t);
    return 0;
  }

  switch (urb->status) {
    case 0:
      t->usbi_transfer.transferred = urb->actual_length;
      status = USB_TRANSFER_COMPLETED;
      break;
    case -EPIPE:
      status = USB_TRANSFER_STALL;
      break;
    case -ETIME:
    case -EPROTO:
    case -EILSEQ:
      status = USB_TRANSFER_ERROR;
      break;
    default:
      status = USB_TRANSFER_ERROR;
      break;
  }

  usbi_handle_transfer_completion(t, (enum usb_transfer_status)status);
  return 0;
}
static int handle_bulk_completion(struct transfer* t, struct usbfs_urb* urb) {
  struct linux_transfer_priv* tpriv = &t->usb_transfer.transfer_priv;
  int num_urbs = tpriv->num_urbs;
  int urb_idx = urb - tpriv->urbS.urbs;
  enum usb_transfer_status status = USB_TRANSFER_COMPLETED;

  if (urb->status == 0 || (urb->status == -EOVERFLOW && urb->actual_length > 0))
    t->usbi_transfer.transferred += urb->actual_length;

  if (tpriv->reap_action != NORMAL) {
    if (urb->status == -ENOENT) {
      if (tpriv->awaiting_discard == 0)
        ;
      else
        tpriv->awaiting_discard--;
    } else if (urb->status == 0) {
      if (tpriv->reap_action == COMPLETED_EARLY)
        ;

      if (tpriv->awaiting_reap == 0)
        ;
      else
        tpriv->awaiting_reap--;
    } else if (urb->status == -EPIPE || urb->status == -EOVERFLOW) {
      if (tpriv->awaiting_reap == 0)
        ;
      else
        tpriv->awaiting_reap--;
    } else {
    }

    if (tpriv->awaiting_reap == 0 && tpriv->awaiting_discard == 0) {
      if (tpriv->reap_action == CANCELLED) {
        usbi_handle_transfer_cancellation(t);
        return 0;
      } else if (tpriv->reap_action == COMPLETED_EARLY) {
        goto out;
      } else {
        status = USB_TRANSFER_ERROR;
        goto out;
      }
    }
    return 0;
  }

  switch (urb->status) {
    case 0:
      break;
    case -EPIPE:
      status = USB_TRANSFER_STALL;
      goto out;
    case -EOVERFLOW:
      status = USB_TRANSFER_OVERFLOW;
      goto out;
    case -ETIME:
    case -EPROTO:
    case -EILSEQ:
      status = USB_TRANSFER_ERROR;
      goto out;
    default:
      status = USB_TRANSFER_ERROR;
      goto out;
  }
  if (urb_idx == num_urbs - 1) {
  } else if (urb->actual_length < urb->buffer_length) {
    struct usb_transfer* transfer = &t->usb_transfer;
    struct linux_device_handle_priv* dpriv = &handle->priv;
    int i;

    tpriv->reap_action = COMPLETED_EARLY;
    for (i = urb_idx + 1; i < tpriv->num_urbs; i++) {
      int r = ioctl(dpriv->fd, IOCTL_USBFS_DISCARDURB, &tpriv->urbS.urbs[i]);
      if (r == 0)
        tpriv->awaiting_discard++;
      else if (errno == EINVAL)
        tpriv->awaiting_reap++;
      else
        ;
    }
    return 0;
  } else {
    return 0;
  }

out:
  usbi_handle_transfer_completion(t, status);
  return 0;
}
static int reap_for_handle() {
  struct linux_device_handle_priv* hpriv = &handle->priv;
  int r;
  struct usbfs_urb* urb;
  struct transfer* t;

  r = ioctl(hpriv->fd, IOCTL_USBFS_REAPURBNDELAY, &urb);
  if (r == -1 && errno == EAGAIN) {
    return 1;
  }
  if (r < 0) {
    if (errno == ENODEV) {
      return USB_ERROR_NO_DEVICE;
    }
    return USB_ERROR_IO;
  }

  t = (struct transfer*)urb->usercontext;

  switch (t->usb_transfer.type) {
    case USB_TRANSFER_TYPE_BULK:
    case USB_TRANSFER_TYPE_INTERRUPT:
      r = handle_bulk_completion(t, urb);
      return r;
    case USB_TRANSFER_TYPE_CONTROL:
      r = handle_control_completion(t, urb);
      return r;
    default:
      return USB_ERROR_OTHER;
  }
}
static int op_handle_events(struct pollfd* fds, nfds_t nfds) {
  int r;
  int i = 0;

  for (i = 0; i < nfds; i++) {
    struct pollfd* pollfd = &fds[i];
    struct linux_device_handle_priv* hpriv = NULL;

    if (!pollfd->revents)
      continue;
    hpriv = &handle->priv;
    if (hpriv->fd == pollfd->fd) {
      r = reap_for_handle();
      if (r == 1 || r == USB_ERROR_NO_DEVICE)
        continue;
      else if (r < 0)
        goto out;
    }
  }
  r = 0;
out:
  return r;
}
const struct usb_func usb_func = {
  open : op_open,
  get_device_descriptor : op_get_device_descriptor,
  get_config_descriptor : op_get_config_descriptor,
  set_configuration : op_set_configuration,
  claim_interface : op_claim_interface,
  submit_transfer : op_submit_transfer,
  cancel_transfer : op_cancel_transfer,
  handle_events : op_handle_events,
};
