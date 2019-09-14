#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QWidget>

class PlotWidget : public QWidget{
public:
    PlotWidget(QWidget *);
    void paintEvent(QPaintEvent *);
    void setBuffer(double *, int sizeBuf);

private:
    int sizeBuffer;
    double *buffer;
    QBrush *background;
    QPainter *p;
};

#endif // PLOTWIDGET_H
