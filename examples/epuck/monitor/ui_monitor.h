/********************************************************************************
** Form generated from reading UI file 'monitor.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MONITOR_H
#define UI_MONITOR_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "plotwidget.h"

QT_BEGIN_NAMESPACE

class Ui_Monitor
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_4;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit_Port;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *checkBox_Sensors;
    QCheckBox *checkBox_Micros;
    QCheckBox *checkBox_Cam;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_2;
    QLineEdit *lineEdit_Height;
    QLabel *label_3;
    QLineEdit *lineEdit_Width;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *pushButton_Connect;
    QPushButton *pushButton_Disconnect;
    QGroupBox *groupBox_7;
    QGridLayout *gridLayout_2;
    QCheckBox *checkBox_Led0;
    QCheckBox *checkBox_Led1;
    QCheckBox *checkBox_Led2;
    QCheckBox *checkBox_Led3;
    QCheckBox *checkBox_Led4;
    QCheckBox *checkBox_Led5;
    QCheckBox *checkBox_Led6;
    QCheckBox *checkBox_Led7;
    QCheckBox *checkBox_Led_front;
    QCheckBox *checkBox_Led_body;
    QLineEdit *lineEdit_MotorLeft;
    QLineEdit *lineEdit_MotorRight;
    QLabel *label_4;
    QLabel *label_5;
    QCheckBox *checkBox_Sound2;
    QCheckBox *checkBox_Sound1;
    QVBoxLayout *verticalLayout_2;
    QSpacerItem *verticalSpacer;
    PlotWidget *plotWidget;
    QScrollArea *scrollCamera;
    QWidget *scrollAreaWidgetContents;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_ConnectionSpeed;
    QLineEdit *lineEdit_ConnectionSpeed;
    QGridLayout *gridLayout_3;
    QGroupBox *groupBox_5;
    QVBoxLayout *verticalLayout_7;
    QLineEdit *lineEdit_Accx;
    QLineEdit *lineEdit_Accy;
    QLineEdit *lineEdit_Accz;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_6;
    QProgressBar *progressBar_AL0;
    QProgressBar *progressBar_AL1;
    QProgressBar *progressBar_AL2;
    QProgressBar *progressBar_AL3;
    QProgressBar *progressBar_AL4;
    QProgressBar *progressBar_AL5;
    QProgressBar *progressBar_AL6;
    QProgressBar *progressBar_AL7;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_3;
    QProgressBar *progressBar_Ground0;
    QProgressBar *progressBar_Ground1;
    QProgressBar *progressBar_Ground2;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_5;
    QProgressBar *progressBar_IR1;
    QProgressBar *progressBar_IR0;
    QProgressBar *progressBar_IR4;
    QProgressBar *progressBar_IR3;
    QProgressBar *progressBar_IR6;
    QProgressBar *progressBar_IR7;
    QProgressBar *progressBar_IR2;
    QProgressBar *progressBar_IR5;
    QGroupBox *groupBox_6;
    QVBoxLayout *verticalLayout;
    QProgressBar *progressBar_GROUND0;
    QProgressBar *progressBar_GROUND1;
    QProgressBar *progressBar_GROUND2;
    QToolBar *mainToolBar;

    void setupUi(QMainWindow *Monitor)
    {
        if (Monitor->objectName().isEmpty())
            Monitor->setObjectName(QStringLiteral("Monitor"));
        Monitor->resize(872, 680);
        centralWidget = new QWidget(Monitor);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout_4 = new QGridLayout(centralWidget);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        lineEdit_Port = new QLineEdit(groupBox);
        lineEdit_Port->setObjectName(QStringLiteral("lineEdit_Port"));

        horizontalLayout->addWidget(lineEdit_Port);


        gridLayout->addLayout(horizontalLayout, 0, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        checkBox_Sensors = new QCheckBox(groupBox);
        checkBox_Sensors->setObjectName(QStringLiteral("checkBox_Sensors"));

        horizontalLayout_2->addWidget(checkBox_Sensors);

        checkBox_Micros = new QCheckBox(groupBox);
        checkBox_Micros->setObjectName(QStringLiteral("checkBox_Micros"));

        horizontalLayout_2->addWidget(checkBox_Micros);

        checkBox_Cam = new QCheckBox(groupBox);
        checkBox_Cam->setObjectName(QStringLiteral("checkBox_Cam"));

        horizontalLayout_2->addWidget(checkBox_Cam);


        gridLayout->addLayout(horizontalLayout_2, 1, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_4->addWidget(label_2);

        lineEdit_Height = new QLineEdit(groupBox);
        lineEdit_Height->setObjectName(QStringLiteral("lineEdit_Height"));

        horizontalLayout_4->addWidget(lineEdit_Height);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_4->addWidget(label_3);

        lineEdit_Width = new QLineEdit(groupBox);
        lineEdit_Width->setObjectName(QStringLiteral("lineEdit_Width"));

        horizontalLayout_4->addWidget(lineEdit_Width);


        gridLayout->addLayout(horizontalLayout_4, 2, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        pushButton_Connect = new QPushButton(groupBox);
        pushButton_Connect->setObjectName(QStringLiteral("pushButton_Connect"));

        horizontalLayout_3->addWidget(pushButton_Connect);

        pushButton_Disconnect = new QPushButton(groupBox);
        pushButton_Disconnect->setObjectName(QStringLiteral("pushButton_Disconnect"));

        horizontalLayout_3->addWidget(pushButton_Disconnect);


        gridLayout->addLayout(horizontalLayout_3, 3, 0, 1, 1);


        gridLayout_4->addWidget(groupBox, 0, 0, 1, 1);

        groupBox_7 = new QGroupBox(centralWidget);
        groupBox_7->setObjectName(QStringLiteral("groupBox_7"));
        groupBox_7->setMinimumSize(QSize(180, 380));
        groupBox_7->setMaximumSize(QSize(180, 16777215));
        gridLayout_2 = new QGridLayout(groupBox_7);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        checkBox_Led0 = new QCheckBox(groupBox_7);
        checkBox_Led0->setObjectName(QStringLiteral("checkBox_Led0"));

        gridLayout_2->addWidget(checkBox_Led0, 0, 0, 1, 1);

        checkBox_Led1 = new QCheckBox(groupBox_7);
        checkBox_Led1->setObjectName(QStringLiteral("checkBox_Led1"));

        gridLayout_2->addWidget(checkBox_Led1, 1, 0, 1, 1);

        checkBox_Led2 = new QCheckBox(groupBox_7);
        checkBox_Led2->setObjectName(QStringLiteral("checkBox_Led2"));

        gridLayout_2->addWidget(checkBox_Led2, 2, 0, 1, 1);

        checkBox_Led3 = new QCheckBox(groupBox_7);
        checkBox_Led3->setObjectName(QStringLiteral("checkBox_Led3"));

        gridLayout_2->addWidget(checkBox_Led3, 3, 0, 1, 1);

        checkBox_Led4 = new QCheckBox(groupBox_7);
        checkBox_Led4->setObjectName(QStringLiteral("checkBox_Led4"));

        gridLayout_2->addWidget(checkBox_Led4, 4, 0, 1, 1);

        checkBox_Led5 = new QCheckBox(groupBox_7);
        checkBox_Led5->setObjectName(QStringLiteral("checkBox_Led5"));

        gridLayout_2->addWidget(checkBox_Led5, 5, 0, 1, 1);

        checkBox_Led6 = new QCheckBox(groupBox_7);
        checkBox_Led6->setObjectName(QStringLiteral("checkBox_Led6"));

        gridLayout_2->addWidget(checkBox_Led6, 6, 0, 1, 1);

        checkBox_Led7 = new QCheckBox(groupBox_7);
        checkBox_Led7->setObjectName(QStringLiteral("checkBox_Led7"));

        gridLayout_2->addWidget(checkBox_Led7, 7, 0, 1, 1);

        checkBox_Led_front = new QCheckBox(groupBox_7);
        checkBox_Led_front->setObjectName(QStringLiteral("checkBox_Led_front"));

        gridLayout_2->addWidget(checkBox_Led_front, 8, 0, 1, 1);

        checkBox_Led_body = new QCheckBox(groupBox_7);
        checkBox_Led_body->setObjectName(QStringLiteral("checkBox_Led_body"));

        gridLayout_2->addWidget(checkBox_Led_body, 9, 0, 1, 1);

        lineEdit_MotorLeft = new QLineEdit(groupBox_7);
        lineEdit_MotorLeft->setObjectName(QStringLiteral("lineEdit_MotorLeft"));
        lineEdit_MotorLeft->setMinimumSize(QSize(0, 23));

        gridLayout_2->addWidget(lineEdit_MotorLeft, 12, 0, 1, 1);

        lineEdit_MotorRight = new QLineEdit(groupBox_7);
        lineEdit_MotorRight->setObjectName(QStringLiteral("lineEdit_MotorRight"));
        lineEdit_MotorRight->setMinimumSize(QSize(0, 23));

        gridLayout_2->addWidget(lineEdit_MotorRight, 14, 0, 1, 1);

        label_4 = new QLabel(groupBox_7);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout_2->addWidget(label_4, 12, 1, 1, 1);

        label_5 = new QLabel(groupBox_7);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout_2->addWidget(label_5, 14, 1, 1, 1);

        checkBox_Sound2 = new QCheckBox(groupBox_7);
        checkBox_Sound2->setObjectName(QStringLiteral("checkBox_Sound2"));

        gridLayout_2->addWidget(checkBox_Sound2, 11, 0, 1, 1);

        checkBox_Sound1 = new QCheckBox(groupBox_7);
        checkBox_Sound1->setObjectName(QStringLiteral("checkBox_Sound1"));

        gridLayout_2->addWidget(checkBox_Sound1, 10, 0, 1, 1);


        gridLayout_4->addWidget(groupBox_7, 0, 1, 2, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        plotWidget = new PlotWidget(centralWidget);
        plotWidget->setObjectName(QStringLiteral("plotWidget"));
        plotWidget->setMinimumSize(QSize(400, 300));
        plotWidget->setMaximumSize(QSize(400, 300));

        verticalLayout_2->addWidget(plotWidget);

        scrollCamera = new QScrollArea(centralWidget);
        scrollCamera->setObjectName(QStringLiteral("scrollCamera"));
        scrollCamera->setMinimumSize(QSize(400, 300));
        scrollCamera->setMaximumSize(QSize(400, 300));
        scrollCamera->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scrollCamera->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scrollCamera->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setEnabled(false);
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 400, 300));
        scrollAreaWidgetContents->setMinimumSize(QSize(400, 300));
        scrollAreaWidgetContents->setMaximumSize(QSize(400, 300));
        scrollCamera->setWidget(scrollAreaWidgetContents);

        verticalLayout_2->addWidget(scrollCamera);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_ConnectionSpeed = new QLabel(centralWidget);
        label_ConnectionSpeed->setObjectName(QStringLiteral("label_ConnectionSpeed"));

        horizontalLayout_5->addWidget(label_ConnectionSpeed);

        lineEdit_ConnectionSpeed = new QLineEdit(centralWidget);
        lineEdit_ConnectionSpeed->setObjectName(QStringLiteral("lineEdit_ConnectionSpeed"));

        horizontalLayout_5->addWidget(lineEdit_ConnectionSpeed);


        verticalLayout_2->addLayout(horizontalLayout_5);


        gridLayout_4->addLayout(verticalLayout_2, 0, 2, 2, 1);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(6);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        groupBox_5 = new QGroupBox(centralWidget);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        groupBox_5->setMaximumSize(QSize(140, 16777215));
        verticalLayout_7 = new QVBoxLayout(groupBox_5);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        lineEdit_Accx = new QLineEdit(groupBox_5);
        lineEdit_Accx->setObjectName(QStringLiteral("lineEdit_Accx"));

        verticalLayout_7->addWidget(lineEdit_Accx);

        lineEdit_Accy = new QLineEdit(groupBox_5);
        lineEdit_Accy->setObjectName(QStringLiteral("lineEdit_Accy"));

        verticalLayout_7->addWidget(lineEdit_Accy);

        lineEdit_Accz = new QLineEdit(groupBox_5);
        lineEdit_Accz->setObjectName(QStringLiteral("lineEdit_Accz"));

        verticalLayout_7->addWidget(lineEdit_Accz);


        gridLayout_3->addWidget(groupBox_5, 1, 0, 1, 1);

        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setMaximumSize(QSize(140, 16777215));
        verticalLayout_6 = new QVBoxLayout(groupBox_4);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        progressBar_AL0 = new QProgressBar(groupBox_4);
        progressBar_AL0->setObjectName(QStringLiteral("progressBar_AL0"));
        progressBar_AL0->setValue(0);

        verticalLayout_6->addWidget(progressBar_AL0);

        progressBar_AL1 = new QProgressBar(groupBox_4);
        progressBar_AL1->setObjectName(QStringLiteral("progressBar_AL1"));
        progressBar_AL1->setValue(0);

        verticalLayout_6->addWidget(progressBar_AL1);

        progressBar_AL2 = new QProgressBar(groupBox_4);
        progressBar_AL2->setObjectName(QStringLiteral("progressBar_AL2"));
        progressBar_AL2->setValue(0);

        verticalLayout_6->addWidget(progressBar_AL2);

        progressBar_AL3 = new QProgressBar(groupBox_4);
        progressBar_AL3->setObjectName(QStringLiteral("progressBar_AL3"));
        progressBar_AL3->setValue(0);

        verticalLayout_6->addWidget(progressBar_AL3);

        progressBar_AL4 = new QProgressBar(groupBox_4);
        progressBar_AL4->setObjectName(QStringLiteral("progressBar_AL4"));
        progressBar_AL4->setValue(0);

        verticalLayout_6->addWidget(progressBar_AL4);

        progressBar_AL5 = new QProgressBar(groupBox_4);
        progressBar_AL5->setObjectName(QStringLiteral("progressBar_AL5"));
        progressBar_AL5->setValue(0);

        verticalLayout_6->addWidget(progressBar_AL5);

        progressBar_AL6 = new QProgressBar(groupBox_4);
        progressBar_AL6->setObjectName(QStringLiteral("progressBar_AL6"));
        progressBar_AL6->setValue(0);

        verticalLayout_6->addWidget(progressBar_AL6);

        progressBar_AL7 = new QProgressBar(groupBox_4);
        progressBar_AL7->setObjectName(QStringLiteral("progressBar_AL7"));
        progressBar_AL7->setValue(0);

        verticalLayout_6->addWidget(progressBar_AL7);


        gridLayout_3->addWidget(groupBox_4, 3, 0, 1, 1);

        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setMaximumSize(QSize(0, 0));
        verticalLayout_3 = new QVBoxLayout(groupBox_3);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        progressBar_Ground0 = new QProgressBar(groupBox_3);
        progressBar_Ground0->setObjectName(QStringLiteral("progressBar_Ground0"));
        progressBar_Ground0->setValue(0);

        verticalLayout_3->addWidget(progressBar_Ground0);

        progressBar_Ground1 = new QProgressBar(groupBox_3);
        progressBar_Ground1->setObjectName(QStringLiteral("progressBar_Ground1"));
        progressBar_Ground1->setValue(0);

        verticalLayout_3->addWidget(progressBar_Ground1);

        progressBar_Ground2 = new QProgressBar(groupBox_3);
        progressBar_Ground2->setObjectName(QStringLiteral("progressBar_Ground2"));
        progressBar_Ground2->setValue(0);

        verticalLayout_3->addWidget(progressBar_Ground2);


        gridLayout_3->addWidget(groupBox_3, 4, 0, 1, 1);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setMaximumSize(QSize(16777215, 16777215));
        gridLayout_5 = new QGridLayout(groupBox_2);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        progressBar_IR1 = new QProgressBar(groupBox_2);
        progressBar_IR1->setObjectName(QStringLiteral("progressBar_IR1"));
        progressBar_IR1->setValue(0);

        gridLayout_5->addWidget(progressBar_IR1, 3, 0, 1, 1);

        progressBar_IR0 = new QProgressBar(groupBox_2);
        progressBar_IR0->setObjectName(QStringLiteral("progressBar_IR0"));
        progressBar_IR0->setValue(0);

        gridLayout_5->addWidget(progressBar_IR0, 0, 0, 1, 1);

        progressBar_IR4 = new QProgressBar(groupBox_2);
        progressBar_IR4->setObjectName(QStringLiteral("progressBar_IR4"));
        progressBar_IR4->setValue(0);

        gridLayout_5->addWidget(progressBar_IR4, 9, 0, 1, 1);

        progressBar_IR3 = new QProgressBar(groupBox_2);
        progressBar_IR3->setObjectName(QStringLiteral("progressBar_IR3"));
        progressBar_IR3->setValue(0);

        gridLayout_5->addWidget(progressBar_IR3, 5, 0, 1, 1);

        progressBar_IR6 = new QProgressBar(groupBox_2);
        progressBar_IR6->setObjectName(QStringLiteral("progressBar_IR6"));
        progressBar_IR6->setValue(0);

        gridLayout_5->addWidget(progressBar_IR6, 12, 0, 1, 1);

        progressBar_IR7 = new QProgressBar(groupBox_2);
        progressBar_IR7->setObjectName(QStringLiteral("progressBar_IR7"));
        progressBar_IR7->setValue(0);

        gridLayout_5->addWidget(progressBar_IR7, 14, 0, 1, 1);

        progressBar_IR2 = new QProgressBar(groupBox_2);
        progressBar_IR2->setObjectName(QStringLiteral("progressBar_IR2"));
        progressBar_IR2->setValue(0);

        gridLayout_5->addWidget(progressBar_IR2, 4, 0, 1, 1);

        progressBar_IR5 = new QProgressBar(groupBox_2);
        progressBar_IR5->setObjectName(QStringLiteral("progressBar_IR5"));
        progressBar_IR5->setValue(0);

        gridLayout_5->addWidget(progressBar_IR5, 10, 0, 1, 1);


        gridLayout_3->addWidget(groupBox_2, 3, 1, 1, 1);

        groupBox_6 = new QGroupBox(centralWidget);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        verticalLayout = new QVBoxLayout(groupBox_6);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        progressBar_GROUND0 = new QProgressBar(groupBox_6);
        progressBar_GROUND0->setObjectName(QStringLiteral("progressBar_GROUND0"));
        progressBar_GROUND0->setValue(0);

        verticalLayout->addWidget(progressBar_GROUND0);

        progressBar_GROUND1 = new QProgressBar(groupBox_6);
        progressBar_GROUND1->setObjectName(QStringLiteral("progressBar_GROUND1"));
        progressBar_GROUND1->setValue(0);

        verticalLayout->addWidget(progressBar_GROUND1);

        progressBar_GROUND2 = new QProgressBar(groupBox_6);
        progressBar_GROUND2->setObjectName(QStringLiteral("progressBar_GROUND2"));
        progressBar_GROUND2->setValue(0);

        verticalLayout->addWidget(progressBar_GROUND2);


        gridLayout_3->addWidget(groupBox_6, 1, 1, 1, 1);


        gridLayout_4->addLayout(gridLayout_3, 1, 0, 1, 1);

        Monitor->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(Monitor);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Monitor->addToolBar(Qt::TopToolBarArea, mainToolBar);

        retranslateUi(Monitor);

        QMetaObject::connectSlotsByName(Monitor);
    } // setupUi

    void retranslateUi(QMainWindow *Monitor)
    {
        Monitor->setWindowTitle(QApplication::translate("Monitor", "Monitor", 0));
        groupBox->setTitle(QApplication::translate("Monitor", "Settings", 0));
        label->setText(QApplication::translate("Monitor", "Port", 0));
        lineEdit_Port->setText(QApplication::translate("Monitor", "/dev/rfcomm0", 0));
        checkBox_Sensors->setText(QApplication::translate("Monitor", "Sensors", 0));
        checkBox_Micros->setText(QApplication::translate("Monitor", "Micros", 0));
        checkBox_Cam->setText(QApplication::translate("Monitor", "Cam", 0));
        label_2->setText(QApplication::translate("Monitor", "Height", 0));
        lineEdit_Height->setText(QApplication::translate("Monitor", "20", 0));
        label_3->setText(QApplication::translate("Monitor", "Width", 0));
        lineEdit_Width->setText(QApplication::translate("Monitor", "20", 0));
        pushButton_Connect->setText(QApplication::translate("Monitor", "Connect", 0));
        pushButton_Disconnect->setText(QApplication::translate("Monitor", "Disconnect", 0));
        groupBox_7->setTitle(QApplication::translate("Monitor", "Motors", 0));
        checkBox_Led0->setText(QApplication::translate("Monitor", "Led 0", 0));
        checkBox_Led1->setText(QApplication::translate("Monitor", "Led 1", 0));
        checkBox_Led2->setText(QApplication::translate("Monitor", "Led 2", 0));
        checkBox_Led3->setText(QApplication::translate("Monitor", "Led 3", 0));
        checkBox_Led4->setText(QApplication::translate("Monitor", "Led 4", 0));
        checkBox_Led5->setText(QApplication::translate("Monitor", "Led 5", 0));
        checkBox_Led6->setText(QApplication::translate("Monitor", "Led 6", 0));
        checkBox_Led7->setText(QApplication::translate("Monitor", "Led 7", 0));
        checkBox_Led_front->setText(QApplication::translate("Monitor", "Led Front", 0));
        checkBox_Led_body->setText(QApplication::translate("Monitor", "Led Body", 0));
        lineEdit_MotorLeft->setText(QApplication::translate("Monitor", "0", 0));
        lineEdit_MotorRight->setText(QApplication::translate("Monitor", "0", 0));
        label_4->setText(QApplication::translate("Monitor", "Motor Left", 0));
        label_5->setText(QApplication::translate("Monitor", "Motor Right", 0));
        checkBox_Sound2->setText(QApplication::translate("Monitor", "Sound 2", 0));
        checkBox_Sound1->setText(QApplication::translate("Monitor", "Sound 1", 0));
        label_ConnectionSpeed->setText(QApplication::translate("Monitor", "Connection Speed [Hz] ", 0));
        groupBox_5->setTitle(QApplication::translate("Monitor", "Accerolometer", 0));
        groupBox_4->setTitle(QApplication::translate("Monitor", "Ambient Light", 0));
        groupBox_3->setTitle(QApplication::translate("Monitor", "Ground", 0));
        groupBox_2->setTitle(QApplication::translate("Monitor", "Prox", 0));
        groupBox_6->setTitle(QApplication::translate("Monitor", "Ground", 0));
    } // retranslateUi

};

namespace Ui {
    class Monitor: public Ui_Monitor {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MONITOR_H
