#include "back.h"

Back::Back(QWidget *parent) : QWidget(parent)
{
}
void Back::paintEvent(QPaintEvent *)
{
    QPixmap background_pic(":/img/background_pic.jpg");
    QPainter background(this);
    background.drawPixmap(0,0,this->width(), this->height(),background_pic);
}
Back::~Back()
{
}
