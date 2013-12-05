#include "monitor.h"
#include "ui_monitor.h"

#include <QTimer>
#include <iostream>
#include <fstream>

using namespace std;


Monitor::Monitor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Monitor)
{
    ui->setupUi(this);

    robot = NULL;
    sensors = new double[EPUCK_SENSORS_MAX];

    for(int i = 0; i<EPUCK_MOTORS_MAX; i++) motors[i]=0;
    for(int i = 0; i<EPUCK_SENSORS_MAX; i++) sensors[i]=0;

    ui->pushButton_Connect->setEnabled(true);
    ui->pushButton_Disconnect->setEnabled(false);
}

Monitor::~Monitor()
{
    delete ui;
}

void Monitor::on_pushButton_Connect_clicked()
{
    ui->pushButton_Connect->setEnabled(false);
    ui->pushButton_Disconnect->setEnabled(true);

    if(robot==NULL){
        conf.CAM_TYPE=1;
        conf.port = ui->lineEdit_Port->text().toStdString();
        conf.MIC_STATE = ui->checkBox_Micros->isChecked();
        conf.CAM_STATE = ui->checkBox_Cam->isChecked();
        conf.SENSOR_STATE = ui->checkBox_Sensors->isChecked();
        conf.CAM_WIDTH = ui->lineEdit_Width->text().toInt();
        conf.CAM_HEIGHT = ui->lineEdit_Height->text().toInt();
        robot = new EPuckBluetooth(conf);

        robot->getNumOfMotSens(numOfSensor, sensorCount, numOfMotor, motorCount);

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(newData()));
        timer->start(200);
    }

}

void Monitor::on_pushButton_Disconnect_clicked()
{
    if(robot!=NULL){
        delete timer;
        delete robot;
        usleep(100000);
        robot=NULL;
    }
    ui->pushButton_Connect->setEnabled(true);
    ui->pushButton_Disconnect->setEnabled(false);
}


void Monitor::newData(){
    robot->getSensors(sensors, sensorCount);

    if(conf.SENSOR_STATE){
        ui->progressBar_IR0->setValue(100*sensors[numOfSensor.IR0]);
        ui->progressBar_IR1->setValue(100*sensors[numOfSensor.IR1]);
        ui->progressBar_IR2->setValue(100*sensors[numOfSensor.IR2]);
        ui->progressBar_IR3->setValue(100*sensors[numOfSensor.IR3]);
        ui->progressBar_IR4->setValue(100*sensors[numOfSensor.IR4]);
        ui->progressBar_IR5->setValue(100*sensors[numOfSensor.IR5]);
        ui->progressBar_IR6->setValue(100*sensors[numOfSensor.IR6]);
        ui->progressBar_IR7->setValue(100*sensors[numOfSensor.IR7]);

        ui->progressBar_GROUND0->setValue(100*sensors[numOfSensor.GROUND0]);
        ui->progressBar_GROUND1->setValue(100*sensors[numOfSensor.GROUND1]);
        ui->progressBar_GROUND2->setValue(100*sensors[numOfSensor.GROUND2]);

        ui->progressBar_AL0->setValue(100*sensors[numOfSensor.AMBIENT_LIGHT0]);
        ui->progressBar_AL1->setValue(100*sensors[numOfSensor.AMBIENT_LIGHT1]);
        ui->progressBar_AL2->setValue(100*sensors[numOfSensor.AMBIENT_LIGHT2]);
        ui->progressBar_AL3->setValue(100*sensors[numOfSensor.AMBIENT_LIGHT3]);
        ui->progressBar_AL4->setValue(100*sensors[numOfSensor.AMBIENT_LIGHT4]);
        ui->progressBar_AL5->setValue(100*sensors[numOfSensor.AMBIENT_LIGHT5]);
        ui->progressBar_AL6->setValue(100*sensors[numOfSensor.AMBIENT_LIGHT6]);
        ui->progressBar_AL7->setValue(100*sensors[numOfSensor.AMBIENT_LIGHT7]);

        ui->lineEdit_Accx->setText(QString().setNum(sensors[numOfSensor.ACCX]));
        ui->lineEdit_Accy->setText(QString().setNum(sensors[numOfSensor.ACCY]));
        ui->lineEdit_Accz->setText(QString().setNum(sensors[numOfSensor.ACCZ]));
    }
    if(conf.MIC_STATE){




#define KORRSIZE 20
        double med0=0, med1=0, med2=0;
        for(int j=-KORRSIZE/2; j<KORRSIZE/2; j++){
            med0+=sensors[numOfSensor.MIC0+50 +j];
            med1+=sensors[numOfSensor.MIC1+50 +j];
            med2+=sensors[numOfSensor.MIC2+50 +j];
        }

        double korr[3][KORRSIZE];
        for(int i=0; i<3*KORRSIZE; i++) korr[0][i]=0;

        for(int i=-+KORRSIZE/2; i<+KORRSIZE/2; i++)for(int j=-KORRSIZE/2; j<KORRSIZE/2; j++){
            korr[0][i+KORRSIZE/2]+= sensors[numOfSensor.MIC0+50 +j]*sensors[numOfSensor.MIC1+50 +j+i]/med0/med1/KORRSIZE;
            korr[1][i+KORRSIZE/2]+= sensors[numOfSensor.MIC1+50 +j]*sensors[numOfSensor.MIC2+50 +j+i]/med1/med2/KORRSIZE;
            korr[2][i+KORRSIZE/2]+= sensors[numOfSensor.MIC2+50 +j]*sensors[numOfSensor.MIC0+50 +j+i]/med2/med0/KORRSIZE;
        }//*/


        ofstream out("korr.dat");
        for(int i=0; i<KORRSIZE; i++) out << i << " " << korr[0][i] << endl;
        out << endl << endl;
        for(int i=0; i<KORRSIZE; i++) out << i << " " << korr[1][i]<< endl;
        out << endl << endl;
        for(int i=0; i<KORRSIZE; i++) out << i << " " << korr[2][i] << endl;
        out.close();
        //*/

        ui->plotWidget->setBuffer(&sensors[numOfSensor.MIC0],300);


    }
    int r,g,b;
    if(conf.CAM_STATE)switch(conf.CAM_TYPE) {
        case 0: {	//grayscale
                    img = QImage(conf.CAM_WIDTH, conf.CAM_HEIGHT, QImage::Format_RGB32);
                    int i=0;
                    for(int y=0; y<conf.CAM_HEIGHT; y++) {
                        for(int x=0; x<conf.CAM_WIDTH; x++) {
                            int r = g = b = (int)sensors[numOfSensor.CAM+i];
                            img.setPixel(x, y, qRgb (r, g, b));
                            i++;
                        }
                    }
                    lblCamera.setPixmap(QPixmap::fromImage(img.scaled(ui->scrollCamera->size(), Qt::KeepAspectRatio)));
                    ui->scrollCamera->setWidget(&lblCamera);
                    break;
                }

        case 1: {	// RGB565 image
                    img = QImage(conf.CAM_WIDTH, conf.CAM_HEIGHT, QImage::Format_RGB16);
                    int i=0;
                    for(int y=0; y<conf.CAM_HEIGHT; y++) {
                        for(int x=0; x<conf.CAM_WIDTH; x++) {
                            int r = (int)((int)sensors[numOfSensor.CAM+i*2])&0xF8;
                            int g = (int)(((int)sensors[numOfSensor.CAM+i*2])&0x07)<<5 | ( ((int)sensors[numOfSensor.CAM+i*2+1])&0xE0)>>3;
                            int b = (int)(((int)sensors[numOfSensor.CAM+i*2+1])&0x1F)<<3;
                            img.setPixel(x, y, qRgb (r, g, b));
                            i++;
                        }
                    }
                    lblCamera.setPixmap(QPixmap::fromImage(img.scaled(ui->scrollCamera->size(), Qt::KeepAspectRatio)));
                    ui->scrollCamera->setWidget(&lblCamera);
                    break;
                }
        default:
                std::cout << "Unknown camera type\n";
                break;
        } //*/
    ui->lineEdit_ConnectionSpeed->setText(QString().setNum(robot->connectionSpeedHz));


    motors[numOfMotor.LED0] = ui->checkBox_Led0->isChecked();
    motors[numOfMotor.LED1] = ui->checkBox_Led1->isChecked();
    motors[numOfMotor.LED2] = ui->checkBox_Led2->isChecked();
    motors[numOfMotor.LED3] = ui->checkBox_Led3->isChecked();
    motors[numOfMotor.LED4] = ui->checkBox_Led4->isChecked();
    motors[numOfMotor.LED5] = ui->checkBox_Led5->isChecked();
    motors[numOfMotor.LED6] = ui->checkBox_Led6->isChecked();
    motors[numOfMotor.LED7] = ui->checkBox_Led7->isChecked();
    motors[numOfMotor.LED_BODY] = ui->checkBox_Led_body->isChecked();
    motors[numOfMotor.LED_FRONT] = ui->checkBox_Led_front->isChecked();
    motors[numOfMotor.MOTOR_LEFT] = ui->lineEdit_MotorLeft->text().toDouble();
    motors[numOfMotor.MOTOR_RIGHT] = ui->lineEdit_MotorRight->text().toDouble();
    motors[numOfMotor.SOUND] = ui->checkBox_Sound1->isChecked()*2;


    robot->setMotors(motors, motorCount);
}
