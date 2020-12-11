#ifndef BAKC_H
#define BACK_H
 
#include <QWidget>
#include <QPainter>
#include "QPixmap"
 
class Back : public QWidget
{
    Q_OBJECT
 
protected:
    void paintEvent(QPaintEvent *);

public:
    Back(QWidget *parent = 0);
    ~Back();
 
};
 
#endif // WIDGET_H
