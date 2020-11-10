#ifndef WIDGET_H
#define WIDGET_H
 
#include <QWidget>
#include <QTimer>
#include <QList>
 
class Widget : public QWidget
{
    Q_OBJECT
 
protected:
    void paintEvent(QPaintEvent *);

public:
    int m_count;
    float x;
    float c;
    QTimer* m_Timer;
    QList<float> xList;
    QList<float> yList;
 
    Widget(QWidget *parent = 0);
    ~Widget();
 
public slots:
    void creatData();
 
};
 
#endif // WIDGET_H
