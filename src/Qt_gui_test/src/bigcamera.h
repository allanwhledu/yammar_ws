#ifndef BIGCAMERA_H
#define BIGCAMERA_H

#include <QWidget>

namespace Ui {
class BigCamera;
}

class BigCamera : public QWidget
{
  Q_OBJECT

public:
  explicit BigCamera(QWidget *parent = nullptr);
  ~BigCamera();

private slots:
  void on_pushButton_clicked();

private:
  Ui::BigCamera *ui;
};

#endif // BIGCAMERA_H
