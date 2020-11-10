#include "Widget.h"
#include <QPainter>
#include <QPointF>
#include <QPen>
#include <qmath.h>
#include <qnamespace.h>
Widget::Widget(QWidget *parent) : QWidget(parent)
{
    x = 0;
    m_count = 0;
    m_Timer = new QTimer(this);
    m_Timer->start(60);

    // c = 1;

    connect(m_Timer,SIGNAL(timeout()),this,SLOT(creatData()));
}
void Widget::creatData()
{
    x += 0.2;
    m_count += 1;
    // if(m_count%2)
    // {
    //     xList.append(x);
    //     yList.append(qSin(x));
    // }
    // else
    // {
    //     xList.append(x);
    //     yList.append(qSin(x));
    //     update();
    // }
    
    // if(m_count%2)
    if(m_count < 50)
    {
	    xList.append(x);
	    // yList.append(2.5/50*x);
	    yList.append(c);
    }
    else {
	    xList.append(x);
	    yList.append(c);
    }

    update();

    if(m_count == 100)
    {
        xList.clear();
        yList.clear();
        m_count = 0;
        x = 0;
    }
}
void Widget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    QPen pen;
    pen.setColor(Qt::red);
    pen.setStyle(Qt::SolidLine);
    pen.setWidthF(0.1);
    painter.setPen(pen);
    painter.setViewport(50, 50, width()-100, height()-100);
    painter.setWindow(-10, 2, 20, -4); // (-10, 2)    (10, -2) 这里的参数是左上角的坐标，长和宽
    painter.fillRect(-10, 2, 20, -4, Qt::white);
    painter.drawLine(QPointF(-10, -1.5), QPointF(10, -1.5));   // x
    painter.drawLine(QPointF(-9.5, 2), QPointF(-9.5, -2));     // y
    for(int i = 0; i < yList.count(); i++)
    {
        if(i == 0)
            painter.drawPoint(QPointF(xList[i]-9.5, yList[i]-1.5));
        else
            painter.drawLine(QPointF(xList[i-1]-9.5, yList[i-1]-1.5), QPointF(xList[i]-9.5, yList[i]-1.5));
    }
}
Widget::~Widget()
{
    
}
