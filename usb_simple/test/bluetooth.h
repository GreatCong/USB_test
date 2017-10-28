#ifndef BLUETOOTH_H
#define BLUETOOTH_H
#ifdef __cplusplus
extern "C"{
#endif
//初始化通讯模块，返回0表示成功，其他表示失败，本接口不支持多线程操作。
unsigned short BluetoothInit();
//向对端发送数据，返回0表示成功，其他表示失败。本接口虽然支持多线程操作但是接口内有锁机制，实际上同时只有一个线程在执行Send操作。
//msgLen：			需要发送的消息缓冲区长度
//msgBuff：			需要发送的消息缓冲区
unsigned short BluetoothSend(unsigned short msgLen,unsigned char *msgBuff);
//接收对端发送的数据，阻塞函数, 返回0表示成功，其他表示失败。不支持多线程操作。
//msgLen：		获取的消息长度, 此参数是传入传出参数，传入的是缓冲区的长度，传出的是消息的长度
//msgBuff：			获取的消息缓冲区
unsigned short BluetoothRecv(unsigned short *msgLen,unsigned char *msgBuff);
//停止蓝牙通讯。
unsigned short BluetoothExit();
#ifdef _cplusplus
}
#endif
#endif 