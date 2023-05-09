#include "serialthread.h"


Serialthread::Serialthread()
{
    connect(this,&QSerialPort::readyRead,[this]
    {
       QByteArray buf = readAll();
       emit SigReceived(buf);
    });

}


void Serialthread:: Start(Settings sets)
{
    //set data
    QSerialPort::setPortName(sets.name);
    QSerialPort::setParity(sets.parity);
    QSerialPort::setBaudRate(sets.baudRate);
    QSerialPort::setDataBits(sets.databits);
    QSerialPort::setStopBits(sets.stopbtis);
    QSerialPort::setFlowControl(sets.flowControl);

    //�򿪴���
    if(QSerialPort::open(QIODevice::ReadWrite))
    {
        emit SigStarted();
    }
    else{
        //��ʧ���˹رմ�1
        emit SigStoped(1);
    }


}

void  Serialthread:: Stop()
{
    //�رմ���
    if(QSerialPort::isOpen())
    {
        QSerialPort::close();
    }

    emit SigStoped(0);

}


void  Serialthread:: Send(QByteArray arr)
{
    if(QSerialPort::isOpen())
    {
        QSerialPort::write(arr);
    }

}


