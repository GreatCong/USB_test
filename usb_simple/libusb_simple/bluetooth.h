#ifndef BLUETOOTH_H
#define BLUETOOTH_H
#ifdef __cplusplus
extern "C"{
#endif
//��ʼ��ͨѶģ�飬����0��ʾ�ɹ���������ʾʧ�ܣ����ӿڲ�֧�ֶ��̲߳�����
unsigned short BluetoothInit();
//��Զ˷������ݣ�����0��ʾ�ɹ���������ʾʧ�ܡ����ӿ���Ȼ֧�ֶ��̲߳������ǽӿ����������ƣ�ʵ����ͬʱֻ��һ���߳���ִ��Send������
//msgLen��			��Ҫ���͵���Ϣ����������
//msgBuff��			��Ҫ���͵���Ϣ������
unsigned short BluetoothSend(unsigned short msgLen,unsigned char *msgBuff);
//���նԶ˷��͵����ݣ���������, ����0��ʾ�ɹ���������ʾʧ�ܡ���֧�ֶ��̲߳�����
//msgLen��		��ȡ����Ϣ����, �˲����Ǵ��봫��������������ǻ������ĳ��ȣ�����������Ϣ�ĳ���
//msgBuff��			��ȡ����Ϣ������
unsigned short BluetoothRecv(unsigned short *msgLen,unsigned char *msgBuff);
//ֹͣ����ͨѶ��
unsigned short BluetoothExit();
#ifdef _cplusplus
}
#endif
#endif 