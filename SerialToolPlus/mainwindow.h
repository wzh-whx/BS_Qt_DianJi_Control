#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QDateTime>
#include <QTimer>
#include <QDebug>
#include <QThread>
#include "Serialthread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void InitUI();

    void InitCOM();

    void closeEvent(QCloseEvent *event);


private slots:
    void on_openButton_clicked();

    void on_sendButton_clicked();

    void on_pushButton_clicked();

    void on_pushButton_3_clicked();
    
    void on_pushButton_4_clicked();
    
    void on_pushButton_11_clicked();
    
    void on_pushButton_12_clicked();
    
signals:
    //����һЩ�źţ�����ȥ�����߳��е� MySerialPort�еĲۺ���
    void  sigStart(Serialthread::Settings  s);
    void  sigStop();
    void  sigSend(QByteArray  data);

 public slots:
    void started();
    void stoped(int status);
    void recieved(QByteArray data);


private:
    Ui::MainWindow *ui;

    //����һ ���Զ����Ĵ���������
    Serialthread m_serial;

    //������������һ���̶߳���
    QThread m_thread;

};
#endif // MAINWINDOW_H
