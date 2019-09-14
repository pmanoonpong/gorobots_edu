#ifndef MONITOR_H
#define MONITOR_H

#include "epuckbluetooth.h"
#include <QtGui/QImage>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLabel>

using namespace lpzrobots;

namespace Ui {
    class Monitor;
}

class Monitor : public QMainWindow
{
    Q_OBJECT

public:
    explicit Monitor(QWidget *parent = 0);
    ~Monitor();

private slots:
    void on_pushButton_Connect_clicked();

    void on_pushButton_Disconnect_clicked();

    void newData();

private:
    Ui::Monitor *ui;
    QImage img;
    QLabel lblCamera;


    EPuckConf conf;
    EPuckBluetooth* robot;
    SensorNumbers numOfSensor;
    MotorNumbers numOfMotor;
    int sensorCount;
    int motorCount;

    double motors[EPUCK_MOTORS_MAX];
    double *sensors;

    QTimer *timer;
};

#endif // MONITOR_H
