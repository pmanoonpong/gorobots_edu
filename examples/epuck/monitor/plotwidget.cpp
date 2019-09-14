#include "plotwidget.h"
#include <QPainter>
#include <iostream>
using namespace std;

PlotWidget::PlotWidget(QWidget *parent):QWidget(parent){
    background = new QBrush(QColor(205,205,205,205));
    p = 0;
    buffer = NULL;
    sizeBuffer = 0;
}


void PlotWidget::paintEvent(QPaintEvent *){
	p = new QPainter(this);
	p->fillRect(this->rect(), *background);

        if(buffer!=0){

            double **micbuffer = new double*[3];
            micbuffer[0]=&buffer[0];
            micbuffer[1]=&buffer[100];
            micbuffer[2]=&buffer[200];

            double medium0, medium1, medium2 = 0;
            for(int i=0; i<100; i++){
                medium0+=micbuffer[0][i];
                medium1+=micbuffer[1][i];
                medium2+=micbuffer[2][i];
            }
            medium0/=100+1;
            medium1/=100;
            medium2/=100-1;

            p->setPen(QColor(0,0,0,255));
            for(int i=0,j=0; i<100; i++){
                p->drawLine((j%(sizeBuffer/3))*4,150-(((double)micbuffer[0][j]-medium0)) ,  (i%(sizeBuffer/3))*4,150-(((double)micbuffer[0][i]-medium0))      );
                j=i;
            }

            p->setPen(QColor(255,0,0,255));
            for(int i=0,j=0; i<100; i++){
                p->drawLine((j%(sizeBuffer/3))*4,150-(((double)micbuffer[1][j]-medium1)) ,  (i%(sizeBuffer/3))*4,150-(((double)micbuffer[1][i]-medium1))      );
                j=i;
            }

            p->setPen(QColor(0,255,0,255));
            for(int i=0,j=0; i<100; i++){
                p->drawLine((j%(sizeBuffer/3))*4,150-(((double)micbuffer[2][j]-medium2)) ,  (i%(sizeBuffer/3))*4,150-(((double)micbuffer[2][i]-medium2))      );
                j=i;
            }


        }

	delete p;
	p = 0;
}

void PlotWidget::setBuffer(double *buf, int sizeBuf){
    if(buffer!=NULL) delete buffer;
    sizeBuffer=sizeBuf;
    buffer = new double[sizeBuffer];
    for(int i=0; i<sizeBuffer; i++)buffer[i]=buf[i];
    update();
}
