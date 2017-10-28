#ifndef FTDI_TYPES_H
#define FTDI_TYPES_H

#include <poll.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define FTDI_MAX_EEPROM_SIZE 256
#define bswap16(x) (((x & 0xff) << 8) | (x >> 8))
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define usb_cpu_to_le16(x) (x)
#define usb_le16_to_cpu(x) (x)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define usb_le16_to_cpu(x) bswap16(x)
#define usb_cpu_to_le16(x) bswap16(x)
#endif
#define VID 0x0403
#define PID 0x6001
#define USB_STAT_PLUGED 1
#define USB_STAT_UNPLUGED 0
#define FREE(x) \
  {             \
    free(x);    \
    x = NULL;   \
  }

struct list_head {
	struct list_head *prev, *next;
};
struct usb_device_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
};
enum ftdi_module_detach_mode {
	AUTO_DETACH_SIO_MODULE = 0,
	DONT_DETACH_SIO_MODULE = 1
};
enum ftdi_chip_type {
	TYPE_AM = 0,
	TYPE_BM = 1,
	TYPE_2232C = 2,
	TYPE_R = 3,
	TYPE_2232H = 4,
	TYPE_4232H = 5,
	TYPE_232H = 6,
	TYPE_230X = 7,
};
struct ftdi_context {
	struct usb_context* usb_ctx;
	struct usb_device_handle* usb_dev;
	int usb_read_timeout;
	int usb_write_timeout;
	enum ftdi_chip_type type;
	int baudrate;
	unsigned char bitbang_enabled;
	unsigned char* readbuffer;
	unsigned int readbuffer_offset;
	unsigned int readbuffer_remaining;
	unsigned int readbuffer_chunksize;
	unsigned int writebuffer_chunksize;
	unsigned int max_packet_size;
	int interface;
	int index;
	int in_ep;
	int out_ep;
	unsigned char bitbang_mode;
	struct ftdi_eeprom* eeprom;
	char* error_str;
	enum ftdi_module_detach_mode module_detach_mode;
};
struct ftdi_transfer_control {
	int completed;
	unsigned char* buf;
	int size;
	int offset;
	struct ftdi_context* ftdi;
	struct usb_transfer* transfer;
};
enum ftdi_interface {
	INTERFACE_ANY = 0,
	INTERFACE_A = 1,
	INTERFACE_B = 2,
	INTERFACE_C = 3,
	INTERFACE_D = 4
};
struct ftdi_eeprom {
	int vendor_id;
	int product_id;
	int initialized_for_connected_device;
	int self_powered;
	int remote_wakeup;
	int is_not_pnp;
	int suspend_dbus7;
	int in_is_isochronous;
	int out_is_isochronous;
	int suspend_pull_downs;
	int use_serial;
	int usb_version;
	int use_usb_version;
	int max_power;
	char* manufacturer;
	char* product;
	char* serial;
	int channel_a_type;
	int channel_b_type;
	int channel_a_driver;
	int channel_b_driver;
	int channel_c_driver;
	int channel_d_driver;
	int channel_a_rs485enable;
	int channel_b_rs485enable;
	int channel_c_rs485enable;
	int channel_d_rs485enable;
	int cbus_function[10];
	int high_current;
	int high_current_a;
	int high_current_b;
	int invert;
	int group0_drive;
	int group0_schmitt;
	int group0_slew;
	int group1_drive;
	int group1_schmitt;
	int group1_slew;
	int group2_drive;
	int group2_schmitt;
	int group2_slew;
	int group3_drive;
	int group3_schmitt;
	int group3_slew;
	int powersave;
	int clock_polarity;
	int data_order;
	int flow_control;
	int size;
	int chip;
	unsigned char buf[FTDI_MAX_EEPROM_SIZE];
	int release_number;
};
#include <pthread.h>
#define list_entry(ptr, type, member) \
  ((type*)((char*)(ptr) - (unsigned long)(&((type*)0L)->member)))
#define list_for_each_entry(pos, head, member)               \
  for (pos = list_entry((head)->next, typeof(*pos), member); \
       &pos->member != (head);                               \
       pos = list_entry(pos->member.next, typeof(*pos), member))

#define list_for_each_entry_safe(pos, n, head, member)        \
  for (pos = list_entry((head)->next, typeof(*pos), member),  \
      n = list_entry(pos->member.next, typeof(*pos), member); \
       &pos->member != (head);                                \
       pos = n, n = list_entry(n->member.next, typeof(*n), member))

#define list_empty(entry) ((entry)->next == (entry))
static inline void list_add_tail(struct list_head* entry,
	struct list_head* head) {
	entry->next = head;
	entry->prev = head->prev;

	head->prev->next = entry;
	head->prev = entry;
}

static inline void list_del(struct list_head* entry) {
	entry->next->prev = entry->prev;
	entry->prev->next = entry->next;
}
static inline void list_init(struct list_head* entry) {
	entry->prev = entry->next = entry;
}

static inline void list_add(struct list_head* entry, struct list_head* head) {
	entry->next = head->next;
	entry->prev = head;

	head->next->prev = entry;
	head->next = entry;
}
struct usb_context {
	int debug;
	int debug_fixed;
	int ctrl_pipe[2];
	struct transfer* flying_transfer;
	unsigned int pollfd_modify;
	pthread_mutex_t pollfd_modify_lock;
	void* fd_cb_user_data;
	pthread_mutex_t events_lock;
	int event_handler_active;
	pthread_mutex_t event_waiters_lock;
	pthread_cond_t event_waiters_cond;
};
enum usb_error {
	USB_SUCCESS = 0,
	USB_ERROR_IO = -1,
	USB_ERROR_INVALID_PARAM = -2,
	USB_ERROR_ACCESS = -3,
	USB_ERROR_NO_DEVICE = -4,
	USB_ERROR_NOT_FOUND = -5,
	USB_ERROR_BUSY = -6,
	USB_ERROR_TIMEOUT = -7,
	USB_ERROR_OVERFLOW = -8,
	USB_ERROR_PIPE = -9,
	USB_ERROR_INTERRUPTED = -10,
	USB_ERROR_NO_MEM = -11,
	USB_ERROR_NOT_SUPPORTED = -12,
	USB_ERROR_OTHER = -99,
};
struct usb_pollfd {
	int fd;
	short events;
};
struct usbi_pollfd {
	struct usb_pollfd pollfd;
	struct list_head list;
};
#define POLLIN 0x001
#define POLLPRI 0x002
#define POLLOUT 0x004
extern struct usb_context* usb_ctx;

struct linux_device_priv {
	char* sysfs_dir;
	unsigned char* dev_descriptor;
	unsigned char* config_descriptor;
};
struct usb_device {
	pthread_mutex_t lock;
	uint8_t bus_number;
	uint8_t device_address;
	uint8_t num_configurations;
	unsigned long session_data;
	char usbfs_dev_path[256];
	struct linux_device_priv dev_priv;
};
struct linux_device_handle_priv {
	int fd;
};
struct usb_device_handle {
	pthread_mutex_t lock;
	unsigned long claimed_interfaces;
	struct list_head list;
	struct linux_device_handle_priv priv;
};
struct usbfs_ctrltransfer {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	uint32_t timeout;
	void* data;
};
enum usb_endpoint_direction {
	USB_ENDPOINT_IN = 0x80,
	USB_ENDPOINT_OUT = 0x00,
};
enum usb_standard_request {
	USB_REQUEST_GET_STATUS = 0x00,
	USB_REQUEST_CLEAR_FEATURE = 0x01,
	USB_REQUEST_SET_FEATURE = 0x03,
	USB_REQUEST_SET_ADDRESS = 0x05,
	USB_REQUEST_GET_DESCRIPTOR = 0x06,
	USB_REQUEST_SET_DESCRIPTOR = 0x07,
	USB_REQUEST_GET_CONFIGURATION = 0x08,
	USB_REQUEST_SET_CONFIGURATION = 0x09,
	USB_REQUEST_GET_INTERFACE = 0x0A,
	USB_REQUEST_SET_INTERFACE = 0x0B,
	USB_REQUEST_SYNCH_FRAME = 0x0C,
};

#define IOCTL_USBFS_CONTROL _IOWR('U', 0, struct usbfs_ctrltransfer)
#define IOCTL_USBFS_BULK _IOWR('U', 2, struct usbfs_bulktransfer)
#define IOCTL_USBFS_RESETEP _IOR('U', 3, unsigned int)
#define IOCTL_USBFS_SETINTF _IOR('U', 4, struct usbfs_setinterface)
#define IOCTL_USBFS_SETCONFIG _IOR('U', 5, unsigned int)
#define IOCTL_USBFS_GETDRIVER _IOW('U', 8, struct usbfs_getdriver)
#define IOCTL_USBFS_SUBMITURB _IOR('U', 10, struct usbfs_urb)
#define IOCTL_USBFS_DISCARDURB _IO('U', 11)
#define IOCTL_USBFS_REAPURB _IOW('U', 12, void*)
#define IOCTL_USBFS_REAPURBNDELAY _IOW('U', 13, void*)
#define IOCTL_USBFS_CLAIMINTF _IOR('U', 15, unsigned int)
#define IOCTL_USBFS_RELEASEINTF _IOR('U', 16, unsigned int)
#define IOCTL_USBFS_CONNECTINFO _IOW('U', 17, struct usbfs_connectinfo)
#define IOCTL_USBFS_IOCTL _IOWR('U', 18, struct usbfs_ioctl)
#define IOCTL_USBFS_HUB_PORTINFO _IOR('U', 19, struct usbfs_hub_portinfo)
#define IOCTL_USBFS_RESET _IO('U', 20)
#define IOCTL_USBFS_CLEAR_HALT _IOR('U', 21, unsigned int)
#define IOCTL_USBFS_DISCONNECT _IO('U', 22)
#define IOCTL_USBFS_CONNECT _IO('U', 23)
struct usb_config_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t MaxPower;
	const struct usb_interface* interface;
	const unsigned char* extra;
	int extra_length;
};
struct discovered_devs {
	size_t len;
	size_t capacity;
	struct usb_device* devices[0];
};
struct usb_func {
	int(*open)();
	int(*get_device_descriptor)(unsigned char* buffer, int* host_endian);
	int(*get_config_descriptor)(uint8_t config_index,
		unsigned char* buffer,
		size_t len,
		int* host_endian);
	int(*set_configuration)(int config);
	int(*claim_interface)(int iface);
	int(*submit_transfer)(struct transfer* t);
	int(*cancel_transfer)(struct transfer* t);
	int(*handle_events)(struct pollfd* fds, nfds_t nfds);
};
struct usb_descriptor_header {
	uint8_t bLength;
	uint8_t bDescriptorType;
};
struct usb_interface {
	const struct usb_interface_descriptor* altsetting;
	int num_altsetting;
};
#define DEVICE_DESC_LENGTH 18

#define USB_MAXENDPOINTS 32
#define USB_MAXINTERFACES 32
#define USB_MAXCONFIG 8

#define DESC_HEADER_LENGTH 2
#define CONFIG_DESC_LENGTH 9
#define INTERFACE_DESC_LENGTH 9
#define ENDPOINT_DESC_LENGTH 7
#define ENDPOINT_AUDIO_DESC_LENGTH 9
enum usb_descriptor_type {
	USB_DT_DEVICE = 0x01,
	USB_DT_CONFIG = 0x02,
	USB_DT_STRING = 0x03,
	USB_DT_INTERFACE = 0x04,
	USB_DT_ENDPOINT = 0x05,
	USB_DT_HID = 0x21,
	USB_DT_REPORT = 0x22,
	USB_DT_PHYSICAL = 0x23,
	USB_DT_HUB = 0x29,
};
struct usb_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
	const struct usb_endpoint_descriptor* endpoint;
	const unsigned char* extra;
	int extra_length;
};
struct usb_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
	uint8_t bRefresh;
	uint8_t bSynchAddress;
	const unsigned char* extra;
	int extra_length;
};
#define USB_DT_DEVICE_SIZE 18
#define USB_DT_CONFIG_SIZE 9
#define USB_DT_INTERFACE_SIZE 9
#define USB_DT_ENDPOINT_SIZE 7
#define USB_DT_ENDPOINT_AUDIO_SIZE 9
#define USB_DT_HUB_NONVAR_SIZE 7

#define USB_ENDPOINT_ADDRESS_MASK 0x0f
#define USB_ENDPOINT_DIR_MASK 0x80
enum usb_transfer_status {
	USB_TRANSFER_COMPLETED,
	USB_TRANSFER_ERROR,
	USB_TRANSFER_TIMED_OUT,
	USB_TRANSFER_CANCELLED,
	USB_TRANSFER_STALL,
	USB_TRANSFER_NO_DEVICE,
	USB_TRANSFER_OVERFLOW,
};

#define USBFS_URB_DISABLE_SPD 1
#define USBFS_URB_ISO_ASAP 2
#define USBFS_URB_QUEUE_BULK 0x10
#define MAX_ISO_BUFFER_LENGTH 32768
#define MAX_BULK_BUFFER_LENGTH 16384
#define MAX_CTRL_BUFFER_LENGTH 4096
enum reap_action {
	NORMAL = 0,
	SUBMIT_FAILED,
	CANCELLED,
	COMPLETED_EARLY,
};
struct linux_transfer_priv {
	union {
		struct usbfs_urb* urbs;
		struct usbfs_urb** iso_urbs;
	} urbS;

	enum reap_action reap_action;
	int num_urbs;
	unsigned int awaiting_reap;
	unsigned int awaiting_discard;

	int iso_packet_offset;
};
struct usbfs_urb {
	unsigned char type;
	unsigned char endpoint;
	int status;
	unsigned int flags;
	void* buffer;
	int buffer_length;
	int actual_length;
	int start_frame;
	int number_of_packets;
	int error_count;
	unsigned int signr;
	void* usercontext;
};
typedef void(*usb_transfer_cb_fn)();
struct usb_transfer {
	uint8_t flags;
	unsigned char endpoint;
	unsigned char type;
	unsigned int timeout;
	enum usb_transfer_status status;
	int length;
	int actual_length;
	usb_transfer_cb_fn callback;
	void* user_data;
	unsigned char* buffer;
	int num_iso_packets;
	struct linux_transfer_priv transfer_priv;
};
struct usbi_transfer {
	int num_iso_packets;
	struct list_head list;
	struct timeval timeout;
	int transferred;
	uint8_t flags;
};
struct transfer {
	struct usbi_transfer usbi_transfer;
	struct usb_transfer usb_transfer;
};
struct usb_control_setup {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};
#define USB_CONTROL_SETUP_SIZE (sizeof(struct usb_control_setup))
enum usb_transfer_flags {
	USB_TRANSFER_SHORT_NOT_OK = 1 << 0,
	USB_TRANSFER_FREE_BUFFER = 1 << 1,
	USB_TRANSFER_FREE_TRANSFER = 1 << 2,
};
enum usb_transfer_type {
	USB_TRANSFER_TYPE_CONTROL = 0,
	USB_TRANSFER_TYPE_ISOCHRONOUS = 1,
	USB_TRANSFER_TYPE_BULK = 2,
	USB_TRANSFER_TYPE_INTERRUPT = 3,
};
#define USBI_TRANSFER_TIMED_OUT (1 << 0)
enum usb_request_type {
	USB_REQUEST_TYPE_STANDARD = (0x00 << 5),
	USB_REQUEST_TYPE_CLASS = (0x01 << 5),
	USB_REQUEST_TYPE_VENDOR = (0x02 << 5),
	USB_REQUEST_TYPE_RESERVED = (0x03 << 5),
};
enum usb_request_recipient {
	USB_RECIPIENT_DEVICE = 0x00,
	USB_RECIPIENT_INTERFACE = 0x01,
	USB_RECIPIENT_ENDPOINT = 0x02,
	USB_RECIPIENT_OTHER = 0x03,
};
enum usbfs_urb_type {
	USBFS_URB_TYPE_ISO = 0,
	USBFS_URB_TYPE_INTERRUPT = 1,
	USBFS_URB_TYPE_CONTROL = 2,
	USBFS_URB_TYPE_BULK = 3,
};
#define FTDI_DEVICE_OUT_REQTYPE \
  (USB_REQUEST_TYPE_VENDOR | USB_RECIPIENT_DEVICE | USB_ENDPOINT_OUT)
#define FTDI_DEVICE_IN_REQTYPE \
  (USB_REQUEST_TYPE_VENDOR | USB_RECIPIENT_DEVICE | USB_ENDPOINT_IN)
#define SIO_RESET 0
#define SIO_MODEM_CTRL 1
#define SIO_SET_FLOW_CTRL 2
#define SIO_SET_BAUD_RATE 3
#define SIO_SET_DATA 4
#define SIO_RESET_REQUEST SIO_RESET
#define SIO_SET_BAUDRATE_REQUEST SIO_SET_BAUD_RATE
#define SIO_SET_DATA_REQUEST SIO_SET_DATA
#define SIO_SET_FLOW_CTRL_REQUEST SIO_SET_FLOW_CTRL
#define SIO_SET_MODEM_CTRL_REQUEST SIO_MODEM_CTRL
#define SIO_POLL_MODEM_STATUS_REQUEST 0x05
#define SIO_SET_EVENT_CHAR_REQUEST 0x06
#define SIO_SET_ERROR_CHAR_REQUEST 0x07
#define SIO_SET_LATENCY_TIMER_REQUEST 0x09
#define SIO_GET_LATENCY_TIMER_REQUEST 0x0A
#define SIO_SET_BITMODE_REQUEST 0x0B
#define SIO_READ_PINS_REQUEST 0x0C
#define SIO_READ_EEPROM_REQUEST 0x90
#define SIO_WRITE_EEPROM_REQUEST 0x91
#define SIO_ERASE_EEPROM_REQUEST 0x92
#define SIO_RESET_SIO 0
#define SIO_RESET_PURGE_RX 1
#define SIO_RESET_PURGE_TX 2

#define SIO_DISABLE_FLOW_CTRL 0x0
#define SIO_RTS_CTS_HS (0x1 << 8)
#define SIO_DTR_DSR_HS (0x2 << 8)
#define SIO_XON_XOFF_HS (0x4 << 8)

#define SIO_SET_DTR_MASK 0x1
#define SIO_SET_DTR_HIGH (1 | (SIO_SET_DTR_MASK << 8))
#define SIO_SET_DTR_LOW (0 | (SIO_SET_DTR_MASK << 8))
#define SIO_SET_RTS_MASK 0x2
#define SIO_SET_RTS_HIGH (2 | (SIO_SET_RTS_MASK << 8))
#define SIO_SET_RTS_LOW (0 | (SIO_SET_RTS_MASK << 8))

#define SIO_RTS_CTS_HS (0x1 << 8)
enum ftdi_parity_type { NONE = 0, ODD = 1, EVEN = 2, MARK = 3, SPACE = 4 };
enum ftdi_stopbits_type { STOP_BIT_1 = 0, STOP_BIT_15 = 1, STOP_BIT_2 = 2 };
enum ftdi_bits_type { BITS_7 = 7, BITS_8 = 8 };
enum ftdi_break_type { BREAK_OFF = 0, BREAK_ON = 1 };
#endif
