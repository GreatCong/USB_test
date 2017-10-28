#include <pthread.h>
#include "ftdi.h"
#include "bluetooth.h"
#define BT_VID 0x0403
#define BT_PID 0x6001
static unsigned char inited = 0;
static unsigned char openned = 0;
static pthread_mutex_t operate_lock;
unsigned short BluetoothInit() {
	if (inited) {
		return 0;
	}
	system("mount -t usbdevfs none /proc/bus/usb");
	if (ftdi_init()) {
		inited = 1;
		pthread_mutex_init(&operate_lock, 0);
	}else{
		inited = 0;
	}
}
static unsigned short BluetoothCheckStat() {
	if (!inited) {
		return 1;
	}
	if (!openned) {
		if (ftdi_open(BT_VID, BT_PID)) {
			openned = 1;
		}
	}
	if (openned) {
		if (!ftdi_get_state()) {
			openned = 0;
		}
	}
	if (!openned) {
		return 2;
	}
	return 0;
}
unsigned short BluetoothSend(unsigned short msgLen, unsigned char *msgBuff) {
	unsigned short r = 0;
	pthread_mutex_lock(&operate_lock);
	if (BluetoothCheckStat()) {
		r = 1;
		goto out;
	}
	if (ftdi_write_data(msgBuff, msgLen) < 0) {
		r = 2;
		goto out;
	}
out:
	pthread_mutex_unlock(&operate_lock);
	return r;
}
unsigned short BluetoothRecv(unsigned short *msgLen, unsigned char *msgBuff) {
	unsigned short r = 0;
	pthread_mutex_lock(&operate_lock);
	if (BluetoothCheckStat()) {
		r = 1;
		goto out;
	}
	if ((*msgLen=ftdi_read_data(msgBuff, msgLen)) < 0) {
		r = 2;
		goto out;
	}
out:
	pthread_mutex_unlock(&operate_lock);
	return r;
}
unsigned short BluetoothExit() {
	pthread_mutex_lock(&operate_lock);
	if (inited) {
		inited = 0;
		openned = 0;
		ftdi_close();
	}
	pthread_mutex_unlock(&operate_lock);
	pthread_mutex_destory(&operate_lock);
}