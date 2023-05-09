#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //注册Settings，以便在  信号槽中作为参数使用
    qRegisterMetaType<Serialthread::Settings>("Serialthread::Settings");
    InitUI();
    InitCOM();


}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::InitUI()
{
    for(int i=1;i<11;i++)
    {
        ui->portName->addItem(QString("COM%1").arg(i));
    }

    ui->baudRate->addItem(QStringLiteral("1200"),QSerialPort::Baud1200   );
    ui->baudRate->addItem(QStringLiteral("2400"),QSerialPort::Baud2400   );
    ui->baudRate->addItem(QStringLiteral("4800"),QSerialPort::Baud4800   );
    ui->baudRate->addItem(QStringLiteral("9600"),QSerialPort::Baud9600   );
    ui->baudRate->addItem(QStringLiteral("19200"),QSerialPort::Baud19200  );
    ui->baudRate->addItem(QStringLiteral("38400"),QSerialPort::Baud38400  );
    ui->baudRate->addItem(QStringLiteral("57600"),QSerialPort::Baud57600  );
    ui->baudRate->addItem(QStringLiteral("115200"),QSerialPort::Baud115200 );
    ui->baudRate->addItem(QStringLiteral("-1"),QSerialPort::UnknownBaud);

    ui->dataBits->addItem(QStringLiteral("5"),QSerialPort::Data5   );
    ui->dataBits->addItem(QStringLiteral("6"),QSerialPort::Data6   );
    ui->dataBits->addItem(QStringLiteral("7"),QSerialPort::Data7   );
    ui->dataBits->addItem(QStringLiteral("8"),QSerialPort::Data8   );
    ui->dataBits->addItem(QStringLiteral("-1"),QSerialPort::UnknownDataBits   );

    ui->parity->addItem(QStringLiteral("No"),QSerialPort::NoParity   );
    ui->parity->addItem(QStringLiteral("Even"),QSerialPort::EvenParity   );
    ui->parity->addItem(QStringLiteral("Odd"),QSerialPort::OddParity   );

    ui->stopBits->addItem("1", QSerialPort::OneStop);
    ui->stopBits->addItem("1.5", QSerialPort::OneAndHalfStop);
    ui->stopBits->addItem("2", QSerialPort::TwoStop);


    ui->flowControl->addItem("None",QSerialPort::NoFlowControl);
    ui->flowControl->addItem("RTS/CTS",QSerialPort::HardwareControl);
    ui->flowControl->addItem("XON/XOFF",QSerialPort::SoftwareControl);


}


//步骤四，使用信号来触发线程中SerialPort对象的一些槽函数
void MainWindow::on_openButton_clicked()
{
    QString text= ui->openButton->text();
    if(text ==  QString::fromLocal8Bit("打开串口"))
    {
         Serialthread::Settings  s;
         s.name= ui->portName->currentText();
         s.baudRate    = (QSerialPort::BaudRate)     ui->baudRate->currentData().toInt();
         s.databits    = (QSerialPort::DataBits)     ui->dataBits->currentData().toInt();
         s.stopbtis    = (QSerialPort::StopBits)     ui->stopBits->currentData().toInt();
         s.parity      = (QSerialPort::Parity)       ui->parity->currentData().toInt();
         s.flowControl = (QSerialPort::FlowControl)  ui->flowControl->currentData().toInt();
        //发信号在线程中打开串口
        emit sigStart(s);
    }
    else
    {
         //发信号在线程中关闭串口
         emit sigStop();
    }


}
/*
void MainWindow::on_sendButton_clicked()
{
    QString strSend= ui->sendTextEdit->toPlainText();
    QByteArray arr= strSend.toUtf8();

    //发信号在线程中写串口数据
    emit sigSend(arr );

}
*/

void MainWindow::InitCOM()
{
    //步骤三
    m_serial.moveToThread(&m_thread);
    m_thread.start();

    //连接响应的信号与槽,  界面主动发送的信号
    connect(this,&MainWindow::sigStart,&m_serial,&Serialthread::Start);
    connect(this,&MainWindow::sigSend,&m_serial,&Serialthread::Send);
    connect(this,&MainWindow::sigStop,&m_serial,&Serialthread::Stop);

    //被动接收的信号
    connect(&m_serial,&Serialthread::SigStarted,this,&MainWindow::started);
    connect(&m_serial,&Serialthread::SigReceived,this,&MainWindow::recieved);
    connect(&m_serial,&Serialthread::SigStoped,this,&MainWindow::stoped);


}



void MainWindow::started()
{
    ui->openButton->setText(QString::fromLocal8Bit("关闭串口"));
    ui->groupBox->setEnabled(false);

}
void MainWindow::stoped(int status)
{
    ui->openButton->setText(QString::fromLocal8Bit("打开串口"));
    ui->groupBox->setEnabled(true);

}
void MainWindow::recieved(QByteArray data)
{
    //字节数组转为字符串
    QString  strText= QString (data);

    //加上时间
     QDateTime current_date_time =QDateTime::currentDateTime();
     QString  t  =current_date_time.toString("yyyy-MM-dd hh:mm:ss.zzz : ");

    //追加到末尾
     ui->recvTextEdit->appendPlainText( t + strText + "\n" );

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    //关闭串口
     emit sigStop();

     //退出线程的消息循环
     m_thread.quit();

     //等待线程结束
     m_thread.wait();
}



uint8_t g_u8_Wise = 0;  //0:使能关  1：顺时针  2：逆时针

uint8_t g_u8_Pos = 0;   //0：位置1   1：位置2


/*
协议：
协议头：            RE
数据组数            1字节
    数据组号：   1字节 eg：01
    数据长度：   1字节
    数据：
    数据组号：   1字节 eg：02
    数据长度：   1字节
    数据：
    .
    .
    .
协议尾：            \r\n(0x0D 0x0A)
*/

//发送
void MainWindow::on_sendButton_clicked()
{
    QString Hear = "RE";
    QString Group_Num = "9";
    QString All_Data = Hear + Group_Num;
    QString Tail = "\r\n";

    //使能关、顺时针、逆时针
    All_Data += "1";    //第1组
    All_Data += "1";    //长度为1
    All_Data += QString::number(g_u8_Wise, 10);     //数据

    //位置1
    All_Data += "2";    //第2组
    All_Data += "5";    //长度为5
    All_Data += QString("%1").arg(ui->lineEdit->text().toInt(), 5, 10, QLatin1Char('0'));    //数据

    //位置1速度
    All_Data += "3";    //第3组
    All_Data += "5";    //长度为5
    if(ui->lineEdit_5->text().mid(0, 1) == "-")
    {
        All_Data += "-";
        All_Data += QString("%1").arg(ui->lineEdit_5->text().mid(1, 10).toInt(), 4, 10, QLatin1Char('0'));    //数据
    }
    else
    {
        All_Data += QString("%1").arg(ui->lineEdit_5->text().toInt(), 5, 10, QLatin1Char('0'));    //数据
    }

    //位置2
    All_Data += "4";    //第4组
    All_Data += "5";    //长度为5
    All_Data += QString("%1").arg(ui->lineEdit_2->text().toInt(), 5, 10, QLatin1Char('0'));    //数据

    //位置2速度
    All_Data += "5";    //第5组
    All_Data += "5";    //长度为5
    if(ui->lineEdit_9->text().mid(0, 1) == "-")
    {
        All_Data += "-";
        All_Data += QString("%1").arg(ui->lineEdit_9->text().mid(1, 10).toInt(), 4, 10, QLatin1Char('0'));    //数据
    }
    else
    {
        All_Data += QString("%1").arg(ui->lineEdit_9->text().toInt(), 5, 10, QLatin1Char('0'));    //数据
    }

    //最大扭矩
    All_Data += "6";    //第6组
    All_Data += "4";    //长度为4
    if(ui->lineEdit_10->text().mid(0, 1) == "-")
    {
        All_Data += "-";
        All_Data += QString("%1").arg(ui->lineEdit_10->text().mid(1, 10).toInt(), 3, 10, QLatin1Char('0'));    //数据
    }
    else
    {
        All_Data += QString("%1").arg(ui->lineEdit_10->text().toInt(), 4, 10, QLatin1Char('0'));    //数据
    }

    //加速时间
    All_Data += "7";    //第7组
    All_Data += "5";    //长度为5
    All_Data += QString("%1").arg(ui->lineEdit_11->text().toInt(), 5, 10, QLatin1Char('0'));    //数据

    //减速时间
    All_Data += "8";    //第8组
    All_Data += "5";    //长度为5
    All_Data += QString("%1").arg(ui->lineEdit_12->text().toInt(), 5, 10, QLatin1Char('0'));    //数据

    //位置1、位置2 选择
    All_Data += "9";    //第9组
    All_Data += "1";    //长度为1
    All_Data += QString::number(g_u8_Pos, 10);     //数据

    //协议尾
    All_Data += Tail;

    m_serial.Send(All_Data.toLatin1().data());   //串口发送

    //RE911 22501 23435 -0456 45043 21550 06786 4-056 75088 08850 00679 11
    //52 45 39 31 31 32 32 35 30 31 32 33 34 33 35 2D 30 34 35 36 34 35 30 34 33 32 31 35 35 30 30 36 37 38 36 34 2D 30 35 36 37 35 30 38 38 30 38 38 35 30 30 30 36 37 39 31 31 0D 0A
}




//使能关
void MainWindow::on_pushButton_clicked()
{
    g_u8_Wise = 0;
}
//顺时针
void MainWindow::on_pushButton_3_clicked()
{
    g_u8_Wise = 1;
}
//逆时针
void MainWindow::on_pushButton_4_clicked()
{
    g_u8_Wise = 2;
}
//位置1
void MainWindow::on_pushButton_11_clicked()
{
    g_u8_Pos = 0;
}
//位置2
void MainWindow::on_pushButton_12_clicked()
{
    g_u8_Pos = 1;
}
