#ifndef FTDI_USB_H
#define FTDI_USB_H
#include "types.h"
#define TIMESPEC_TO_TIMEVAL(tv, ts)       \
  {                                       \
    (tv)->tv_sec = (ts)->tv_sec;          \
    (tv)->tv_usec = (ts)->tv_nsec / 1000; \
  }
extern const struct usb_func usb_func;
extern struct usb_context* usb_ctx;
extern struct usb_device* dest_device;
extern struct usb_device_handle* handle;
extern struct transfer* bulk_transfer;
extern struct transfer* ctrl_transfer;
#ifdef __cplusplus
extern "C" {
#endif
	int usb_fetch(int vid, int pid);
	void usb_close();
	int usb_mem_init();
	int usb_open();
	int get_device_descriptor(struct usb_device_descriptor* desc);
	int get_config_descriptor(uint8_t config_index,
		struct usb_config_descriptor* config);
	int get_configuration(int* config);
	int set_configuration(int config);
	int claim_interface(int interface_number);
	int usb_get_device_state();
#ifdef __cplusplus
}
#endif
int bulk_transfer_func(unsigned char endpoint,
                       unsigned char* data,
                       int length,
                       int* transferred,
                       unsigned int timeout);
int control_transfer_func(uint8_t bmRequestType,
                          uint8_t bRequest,
                          uint16_t wValue,
                          uint16_t wIndex,
                          unsigned char* data,
                          uint16_t wLength,
                          unsigned int timeout);
static inline int get_string_descriptor(uint8_t desc_index,
                                        uint16_t langid,
                                        unsigned char* data,
                                        int length) {
  return control_transfer_func(USB_ENDPOINT_IN, USB_REQUEST_GET_DESCRIPTOR,
                               (USB_DT_STRING << 8) | desc_index, langid, data,
                               length, 1000);
}
static inline void fill_control_setup(unsigned char* buffer,
                                      uint8_t bmRequestType,
                                      uint8_t bRequest,
                                      uint16_t wValue,
                                      uint16_t wIndex,
                                      uint16_t wLength) {
  struct usb_control_setup* setup = (struct usb_control_setup*)buffer;
  setup->bmRequestType = bmRequestType;
  setup->bRequest = bRequest;
  setup->wValue = usb_cpu_to_le16(wValue);
  setup->wIndex = usb_cpu_to_le16(wIndex);
  setup->wLength = usb_cpu_to_le16(wLength);
}
static inline void fill_control_transfer(unsigned char* buffer,
                                         usb_transfer_cb_fn callback,
                                         void* user_data,
                                         unsigned int timeout) {
  struct usb_control_setup* setup = (struct usb_control_setup*)buffer;
  ctrl_transfer->usb_transfer.endpoint = 0;
  ctrl_transfer->usb_transfer.type = USB_TRANSFER_TYPE_CONTROL;
  ctrl_transfer->usb_transfer.timeout = timeout;
  ctrl_transfer->usb_transfer.buffer = buffer;
  if (setup)
    ctrl_transfer->usb_transfer.length =
        USB_CONTROL_SETUP_SIZE + usb_le16_to_cpu(setup->wLength);
  ctrl_transfer->usb_transfer.user_data = user_data;
  ctrl_transfer->usb_transfer.callback = callback;
}
static inline void fill_bulk_transfer(unsigned char endpoint,
                                      unsigned char* buffer,
                                      int length,
                                      usb_transfer_cb_fn callback,
                                      void* user_data,
                                      unsigned int timeout) {
  bulk_transfer->usb_transfer.endpoint = endpoint;
  bulk_transfer->usb_transfer.type = USB_TRANSFER_TYPE_BULK;
  bulk_transfer->usb_transfer.timeout = timeout;
  bulk_transfer->usb_transfer.buffer = buffer;
  bulk_transfer->usb_transfer.length = length;
  bulk_transfer->usb_transfer.user_data = user_data;
  bulk_transfer->usb_transfer.callback = callback;
}
static inline void* transfer_get_os_priv(struct usbi_transfer* transfer) {
  return (void*)(((char*)transfer) + sizeof(struct usbi_transfer) +
                 sizeof(struct usb_transfer));
}
#endif
