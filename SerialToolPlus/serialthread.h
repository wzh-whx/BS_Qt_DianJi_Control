#ifndef SERIALTHREAD_H
#define SERIALTHREAD_H

#include<QSerialPort>
#include<QDebug>
#include<QThread>


class Serialthread : public QSerialPort
{
    Q_OBJECT
public:
    Serialthread();

    struct Settings{
        QString name;
        BaudRate baudRate;
        DataBits databits;
        Parity parity;
        StopBits stopbtis;
        FlowControl flowControl;
    };

public slots:

    //�򿪴���
    void  Start(Settings sets);
    void  Stop();
    void  Send(QByteArray arr);

signals: //�����洫�ݵ�һЩ�ź�
    void SigStarted(); //���ڴ���
    void SigStoped(int status); //����ֹͣ��
    void SigReceived(QByteArray data); //�����յ�������



};

#endif // SERIALTHREAD_H
