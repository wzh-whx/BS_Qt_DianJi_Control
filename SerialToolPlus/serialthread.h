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

    //打开串口
    void  Start(Settings sets);
    void  Stop();
    void  Send(QByteArray arr);

signals: //向外面传递的一些信号
    void SigStarted(); //串口打开了
    void SigStoped(int status); //串口停止了
    void SigReceived(QByteArray data); //串口收到数据了



};

#endif // SERIALTHREAD_H
