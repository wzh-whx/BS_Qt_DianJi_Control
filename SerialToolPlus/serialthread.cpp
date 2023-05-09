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

    //打开串口
    if(QSerialPort::open(QIODevice::ReadWrite))
    {
        emit SigStarted();
    }
    else{
        //打开失败了关闭传1
        emit SigStoped(1);
    }


}

void  Serialthread:: Stop()
{
    //关闭串口
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


