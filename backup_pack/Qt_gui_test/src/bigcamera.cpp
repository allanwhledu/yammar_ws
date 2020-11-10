#include "bigcamera.h"
#include "ui_bigcamera.h"

BigCamera::BigCamera(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::BigCamera)
{
  ui->setupUi(this);
}

BigCamera::~BigCamera()
{
  delete ui;
}

void BigCamera::on_pushButton_clicked()
{
    this->deleteLater();
}
