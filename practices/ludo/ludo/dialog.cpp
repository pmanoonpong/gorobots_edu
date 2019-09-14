#include "dialog.h"
#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setBackgroundBrush(QBrush(QColor(240,240,239)));

    diceBG = scene->addRect(0,-150,100,100,QPen(Qt::black,3,Qt::SolidLine,Qt::RoundCap, Qt::RoundJoin),QBrush(Qt::green));
    diceRoll = scene->addSimpleText(QString::number(0),QFont("Courier", 72, QFont::Bold, true));
    diceRoll->setPos(25,-150);
    // Colors
    std::vector<std::pair<QColor,QColor> >base_colors {
        std::make_pair(QColor(92,170,119),QColor(185,219,125)), //G
        std::make_pair(QColor(237,235,89),QColor(237,234,138)),  //Y
        std::make_pair(QColor(92,93,170),QColor(111,111,170)),  //B
        std::make_pair(QColor(237,57,60),QColor(237,114,125))   //R
    };

    QBrush white(Qt::white);
    QPen blackPen(Qt::black);
    blackPen.setWidth(1);


    // Cross
    scene->addRect(415,-155,160,960,blackPen,QBrush(QColor(195,195,194)));
    scene->addRect(15,245,960,160,blackPen,QBrush(QColor(195,195,194)));
    scene->addRect(415,245,160,160,QPen(QColor(195,195,194)),QBrush(QColor(195,195,194))); //clean center

    // Goal stretch
    scene->addRect(50,290,350,70,blackPen,QBrush(base_colors[0].first));
    scene->addRect(460,-120,70,350,blackPen,QBrush(base_colors[1].first));
    scene->addRect(590,290,350,70,blackPen,QBrush(base_colors[2].first));
    scene->addRect(460,420,70,350,blackPen,QBrush(base_colors[3].first));

    int x_pos = -10; //start place for green
    int y_pos = 220;
    int offset = 70;
    int small_offset = 50;
    int large_offset = 80;

    //home fields
    home_fields.push_back(QPointF(0,0));
    home_fields.push_back(QPointF(630,-170));
    home_fields.push_back(QPointF(800,445));
    home_fields.push_back(QPointF(190,630));
    for(size_t f = 0; f < home_fields.size(); ++f){
        addHomeField(home_fields[f].x(),home_fields[f].y(),QBrush(base_colors[f].first));
    }

    // Playing fields
    std::vector<std::pair<char,char> > directions{std::make_pair(1,-1),std::make_pair(1,1),std::make_pair(-1,1),std::make_pair(-1,-1) };
    for(size_t d =0; d < directions.size(); ++d){
        for(int i=0; i<5;++i){
            if(d % 2 == 0)
                x_pos += directions[d].first * offset;
            else
                y_pos += directions[d].second * offset;
            fieldPos.push_back(QPointF(x_pos,y_pos));
        }
        x_pos += directions[d].first * small_offset;
        y_pos += directions[d].second * small_offset;
        for(int i=0; i<5;++i){
            fieldPos.push_back(QPointF(x_pos,y_pos));
            if(d % 2 == 0)
                y_pos += directions[d].second * offset;
            else
                x_pos += directions[d].first * offset;
        }
        for(int i=0; i<2;++i){
            fieldPos.push_back(QPointF(x_pos,y_pos));
            if(d % 2 == 0)
                x_pos += directions[d].first * large_offset;
            else
                y_pos += directions[d].second * large_offset;
        }
        fieldPos.push_back(QPointF(x_pos,y_pos));
    }

    //goal stretches
    for(int x=60; x<=340; x+=offset)
        fieldPos.push_back(QPointF(x,300));
    for(int y=-110; y<=170; y+=offset)
        fieldPos.push_back(QPointF(470,y));
    for(int x=880; x>=600; x-=offset)
        fieldPos.push_back(QPointF(x,300));
    for(int y=710; y>=430; y-=offset)
        fieldPos.push_back(QPointF(470,y));

    QImage globe_img("../globe.png");//http://www.clker.com/clipart-world-black-and-white.html
    QImage star_img("../star.png");  //http://www.clker.com/clipart-2568.html
//    QGraphicsPixmapItem globe( QPixmap::fromImage(QImage("../globe.png")));
//    QGraphicsPixmapItem star( QPixmap::fromImage(QImage("../star.png")));
    for(size_t c = 0; c < base_colors.size(); ++c){
        scene->addEllipse(fieldPos[0+13*c].x(),fieldPos[0+13*c].y(),50,50,QPen(base_colors[c].first),QBrush(base_colors[c].second));
        for(int i=1; i < 13; ++i){
            if(i == 8){
                QGraphicsPixmapItem * globe = new QGraphicsPixmapItem( QPixmap::fromImage(globe_img));
                globe->setPos(fieldPos[i+13*c]);
                globe->setScale(0.5);
                scene->addItem(globe);
            } else if(i == 5 || i == 11){
                QGraphicsPixmapItem * star = new QGraphicsPixmapItem( QPixmap::fromImage(star_img));
                star->setPos(fieldPos[i+13*c]);
                star->setScale(0.5);
                scene->addItem(star);
            } else {
                scene->addEllipse(fieldPos[i+13*c].x(),fieldPos[i+13*c].y(),50,50,blackPen,white);
            }
        }
    }
    for(size_t g = 52; g < fieldPos.size(); ++g){
        scene->addEllipse(fieldPos[g].x(),fieldPos[g].y(),50,50,blackPen,white);
    }
    create_graphic_players();
    std::vector<int> init_pos(16,-1);
    update_graphics(init_pos);
}

void Dialog::update_graphics(std::vector<int> player_positions){
    QPointF p;
    for(size_t i = 0; i < player_positions.size(); ++i){
        if(player_positions[i] == -1){
            p = home_fields[i / 4];
            if(i % 4 == 0)
                graphic_player[i]->setPos(p.x()+65 ,p.y()+15 );
            else if(i % 4 == 1)
                graphic_player[i]->setPos(p.x()+65 ,p.y()+115);
            else if(i % 4 == 2)
                graphic_player[i]->setPos(p.x()+15 ,p.y()+65 );
            else if(i % 4 == 3)
                graphic_player[i]->setPos(p.x()+115,p.y()+65 );

        } else if(player_positions[i] == 99){
            if(i/4 == 0){
                if(i % 4 == 0)      graphic_player[i]->setPos(405,300); //left
                else if(i % 4 == 1) graphic_player[i]->setPos(405,270);
                else if(i % 4 == 2) graphic_player[i]->setPos(405,330);
                else if(i % 4 == 3) graphic_player[i]->setPos(435,300);
            } else if(i/4 == 1){
                if(i % 4 == 0)      graphic_player[i]->setPos(470,235); //up
                else if(i % 4 == 1) graphic_player[i]->setPos(440,235);
                else if(i % 4 == 2) graphic_player[i]->setPos(500,235);
                else if(i % 4 == 3) graphic_player[i]->setPos(470,265);
            } else if(i/4 == 2){
                if(i % 4 == 0)      graphic_player[i]->setPos(535,300); //right
                else if(i % 4 == 1) graphic_player[i]->setPos(535,270);
                else if(i % 4 == 2) graphic_player[i]->setPos(535,330);
                else if(i % 4 == 3) graphic_player[i]->setPos(505,300);
            } else if(i/4 == 3){
                if(i % 4 == 0)      graphic_player[i]->setPos(470,365); //down
                else if(i % 4 == 1) graphic_player[i]->setPos(440,365);
                else if(i % 4 == 2) graphic_player[i]->setPos(500,365);
                else if(i % 4 == 3) graphic_player[i]->setPos(470,335);
            }
        } else {
            graphic_player[i]->setPos(fieldPos[player_positions[i]]);
        }
    }
    ui->graphicsView->repaint();
}

void Dialog::create_graphic_players(){
    graphic_player.clear();
    QBrush piece;
    QPen blackPen(Qt::black);
    blackPen.setWidth(1);
    for(int c = 0; c<4; ++c){
        if(c == 0){
            piece = QBrush(QColor(Qt::green));
        } else if(c == 1){
            piece = QBrush(QColor(Qt::yellow));
        } else if(c == 2){
            piece = QBrush(QColor(Qt::blue));
        } else if(c == 3){
            piece = QBrush(QColor(Qt::red));
        }
        for(int i = 0; i<4; ++i){
            graphic_player.push_back(scene->addEllipse(5,5,40,40,blackPen,piece));
        }
    }
}


Dialog::~Dialog()
{
    delete ui;
}

void Dialog::showEvent(QShowEvent *) {
    ui->graphicsView->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
}

void Dialog::resizeEvent(QResizeEvent *){
    ui->graphicsView->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
}

void Dialog::get_winner(int color){
    scene->addRect(0,500,1000,200,QPen(Qt::black,3,Qt::SolidLine,Qt::RoundCap, Qt::RoundJoin),QBrush(active_color));
    QGraphicsSimpleTextItem * win = scene->addSimpleText(QString("Winner is found!"),QFont("Courier", 72, QFont::Bold, true));
    win->setPos(50,550);
}

void Dialog::get_color(int color){
    switch(color){
        case 0:
            active_color = Qt::green;
            break;
        case 1:
            active_color = Qt::yellow;
            break;
        case 2:
            active_color = Qt::blue;
            break;
        case 3:
            active_color = Qt::red;
        default:
            break;
    }
}

void Dialog::get_dice_result(int dice){
    current_dice_roll = dice;
    diceBG->setBrush(active_color);
    diceRoll->setText(QString::number(current_dice_roll));
    ui->graphicsView->repaint();
}

void Dialog::addHomeField(int x, int y,QBrush brush){
    QBrush whiteBrush(Qt::white);
    QPen blackPen(Qt::black);
    blackPen.setWidth(1);

    scene->addEllipse(x,y,180,180,blackPen,brush);
    scene->addEllipse(x+65 ,y+15 ,50,50,blackPen,whiteBrush);
    scene->addEllipse(x+65 ,y+115,50,50,blackPen,whiteBrush);
    scene->addEllipse(x+15 ,y+65 ,50,50,blackPen,whiteBrush);
    scene->addEllipse(x+115,y+65 ,50,50,blackPen,whiteBrush);
}
