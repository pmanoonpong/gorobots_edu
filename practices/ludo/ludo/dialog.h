#ifndef DIALOG_H
#define DIALOG_H

#include <iostream>
#include <vector>
#include <utility> //std::pair
#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QGraphicsScene>
#include "game.h"
#include <QGraphicsEllipseItem>
#include <QPixmap>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
private:
    QColor active_color;
    int current_dice_roll;
    void addHomeField(int,int,QBrush);
    void showEvent(QShowEvent *);
    void resizeEvent(QResizeEvent *);
    Ui::Dialog *ui;
    QGraphicsScene *scene;
    QGraphicsEllipseItem *ellipse;
    QGraphicsRectItem *rectangle;
    void create_graphic_players();
    std::vector<QGraphicsEllipseItem *> graphic_player;
    QGraphicsSimpleTextItem * diceRoll;
    QGraphicsRectItem * diceBG;
    std::vector<QPointF> home_fields;
    std::vector<QPointF> fieldPos;
public slots:
    void update_graphics(std::vector<int> player_positions);
    void get_color(int color);
    void get_dice_result(int dice);
    void get_winner(int color);
};

#endif // DIALOG_H
