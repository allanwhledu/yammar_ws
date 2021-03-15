#include "mainwindow.h"
#include "bigcamera.h"
#include "ui_mainwindow.h"
#include <Widget.h>
#include <qnamespace.h>
#include <QPainter>
#include <QPixmap>
#include <MyDrawImage.h>
#include <QImageReader>
#include <QDebug>

float carspeed = 0;
int position =0;

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow),
        qnode(argc, argv) {
    ui->setupUi(this);

    // Widget paint_line;
    // paint_line.show();
    // 实际上这里不需要做任何的事情，因为promote的widget出来就是自己显示的。
    ui->widget->show();
//    ui->widget_2->show();
    // 在主函数里仅仅可以强制隐藏。
    // ui->widget->hide();
    // displayChart();

    // QProgressBar *m_pLeftToRightProBar = new QProgressBar(this);
    // m_pLeftToRightProBar->setOrientation(Qt::Horizontal);  // 水平方向
    // m_pLeftToRightProBar->setMinimum(0);  // 最小值
    // m_pLeftToRightProBar->setMaximum(100);  // 最大值
    // m_pLeftToRightProBar->setValue(50);  // 当前进度



    std::cout << "init ros node..." << std::endl;
    qnode.init();

    ui->progressBar->setMinimum(0);
    ui->progressBar->setMaximum(5000);
    ui->progressBar->setValue(0);

    ui->progressBar_2->setMinimum(0);
    ui->progressBar_2->setMaximum(5000);
    ui->progressBar_2->setValue(0);

    ui->progressBar_3->setMinimum(0);
    ui->progressBar_3->setMaximum(30);
    ui->progressBar_3->setValue(0);

    ui->progressBar1->setMinimum(0);
    ui->progressBar1->setMaximum(2000);
    ui->progressBar1->setValue(1000);

    ui->progressBar1_2->setMinimum(0);
    ui->progressBar1_2->setMaximum(3000);
    ui->progressBar1_2->setValue(1000);

    ui->progressBar1_3->setMinimum(0);
    ui->progressBar1_3->setMaximum(3000);
    ui->progressBar1_3->setValue(2500);

    ui->progressBar1_4->setMinimum(0);
    ui->progressBar1_4->setMaximum(3000);
    ui->progressBar1_4->setValue(0);

    ui->progressBar1_5->setMinimum(0);
    ui->progressBar1_5->setMaximum(3000);
    ui->progressBar1_5->setValue(0);

    ui->progressBar1_6->setMinimum(0);
    ui->progressBar1_6->setMaximum(3000);
    ui->progressBar1_6->setValue(0);

    ui->progressBar1_7->setMinimum(0);
    ui->progressBar1_7->setMaximum(3000);
    ui->progressBar1_7->setValue(0);

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(loggingCamera()), this, SLOT(updateLogcamera()));
    QObject::connect(&qnode, SIGNAL(logging_leader_line_error()), this, SLOT(updateText()));
    QObject::connect(&qnode, SIGNAL(loggingChart()), this, SLOT(displayChart()));
    QObject::connect(&qnode, SIGNAL(logging_REEL_speed()), this, SLOT(updateREEL()));
    QObject::connect(&qnode, SIGNAL(logging_CB_speed()), this, SLOT(updateCB()));
    QObject::connect(&qnode, SIGNAL(logging_PF_speed()), this, SLOT(updatePF()));
    QObject::connect(&qnode, SIGNAL(logging_FH_speed()), this, SLOT(updateFH()));
    QObject::connect(&qnode, SIGNAL(logging_REEL_current()), this, SLOT(updateREEL_current()));
    QObject::connect(&qnode, SIGNAL(logging_CB_current()), this, SLOT(updateCB_current()));
    QObject::connect(&qnode, SIGNAL(logging_PF_current()), this, SLOT(updatePF_current()));
    QObject::connect(&qnode, SIGNAL(logging_FH_current()), this, SLOT(updateFH_current()));
    QObject::connect(&qnode, SIGNAL(logging_is_obstacle()), this, SLOT(update_is_obstacle()));
    QObject::connect(&qnode, SIGNAL(logging_no_obstacle()), this, SLOT(update_no_obstacle()));
    QObject::connect(&qnode, SIGNAL(logging_reap_height1()), this, SLOT(update_reap_height1()));
    QObject::connect(&qnode, SIGNAL(logging_reap_height2()), this, SLOT(update_reap_height2()));
    QObject::connect(&qnode, SIGNAL(logging_torque()), this, SLOT(update_torque()));
    // QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    qDebug() << "Supported formats:" << QImageReader::supportedImageFormats();

}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_pushButtonConnect_clicked() {
    std::cout << "init button pressed." << std::endl;
    qnode.init();
}

void MainWindow::on_pushButtonSend_clicked() {
    qnode.ros_test("I changed this world!");
}

void MainWindow::updateLogcamera() {
    displayMat(qnode.image);
}

void MainWindow::updateText() {
  displayText(qnode.leader_line_error_string, qnode.height_string);
  if (qnode.leader_line_error < -2) {
      ui->go_left_button->setStyleSheet("background-color: red");
      ui->go_right_button->setStyleSheet("background-color: white");
  } else if (qnode.leader_line_error > 2) {
      ui->go_left_button->setStyleSheet("background-color: white");
      ui->go_right_button->setStyleSheet("background-color: red");
  } else {
      ui->go_left_button->setStyleSheet("background-color: white");
      ui->go_right_button->setStyleSheet("background-color: white");
  }

  if (qnode.height_value < -2) {
      ui->height_button->setStyleSheet("background-color: red");
      ui->down_button->setStyleSheet("background-color: white");
  } else if (qnode.height_value > 2) {
      ui->height_button->setStyleSheet("background-color: white");
      ui->down_button->setStyleSheet("background-color: red");
  } else {
      ui->height_button->setStyleSheet("background-color: white");
      ui->down_button->setStyleSheet("background-color: white");
  }
}

void MainWindow::displayChart() {
    ui->widget->c = qnode.chart;
}

void MainWindow::updateREEL() {
    QString qstr;
    qstr = QString::number(qnode.REEL_speed);
    ui->lineEdit_50->setText(qstr);
    ui->progressBar1_4->setValue(qnode.REEL_speed);

    // for(int i=1;i<qnode.REEL_speed;i++)
    // {
    //         ui->progressBar1_4->setValue(i);
    // }
}

void MainWindow::updateCB() {
    QString qstr;
    qstr = QString::number(qnode.CB_speed);
    ui->lineEdit_54->setText(qstr);
    ui->progressBar1_5->setValue(qnode.CB_speed);

    // for(int i=1;i<qnode.REEL_speed;i++)
    // {
    //         ui->progressBar1_4->setValue(i);
    // }
}

void MainWindow::updatePF() {
    QString qstr;
    qstr = QString::number(qnode.PF_speed);
    ui->lineEdit_55->setText(qstr);
    ui->progressBar1_6->setValue(qnode.PF_speed);

    // for(int i=1;i<qnode.REEL_speed;i++)
    // {
    //         ui->progressBar1_4->setValue(i);
    // }
}

void MainWindow::updateFH() {
    QString qstr;
    qstr = QString::number(qnode.FH_speed);
    ui->lineEdit_56->setText(qstr);
    ui->progressBar1_7->setValue(qnode.FH_speed);

    // for(int i=1;i<qnode.REEL_speed;i++)
    // {
    //         ui->progressBar1_4->setValue(i);
    // }
}

//// update current
void MainWindow::updateREEL_current() {
    QString qstr;
    qstr = QString::number(qnode.REEL_current);
    ui->lineEdit_9->setText(qstr);
}

void MainWindow::updateCB_current() {
    QString qstr;
    qstr = QString::number(qnode.CB_current);
    ui->lineEdit_10->setText(qstr);
}

void MainWindow::updatePF_current() {
    QString qstr;
    qstr = QString::number(qnode.PF_current);
    ui->lineEdit_11->setText(qstr);
}

void MainWindow::updateFH_current() {
    QString qstr;
    qstr = QString::number(qnode.FH_current);
    ui->lineEdit_12->setText(qstr);
}
//// update current

void MainWindow::update_is_obstacle() {
//  std::cout<<"obstacle!"<<std::endl;
    ui->pushButton_10->setStyleSheet("background-color: red");
}

void MainWindow::update_no_obstacle() {
//  std::cout<<"obstacle!"<<std::endl;
    ui->pushButton_10->setStyleSheet("background-color: green");
}

void MainWindow::update_reap_height1() {
    QString qheight;
    qheight = QString::number(qnode.reap_height1);
    ui->lineEdit_41->setText(qheight);
    ui->progressBar->setValue(qnode.reap_height1);
}

void MainWindow::update_reap_height2() {
    QString qheight;
    qheight = QString::number(qnode.reap_height2);
    ui->lineEdit_51->setText(qheight);
    ui->progressBar_2->setValue(qnode.reap_height2);
}

void MainWindow::update_torque() {
    QString qtorque;
    qtorque = QString::number(qnode.torque);
    ui->lineEdit_52->setText(qtorque);
    ui->progressBar_3->setValue(10 * qnode.torque);
}

void MainWindow::displayMat(const QImage &image) {
    qimage_mutex_.lock();
    qimage_ = image.copy();
    ui->label_camera->setPixmap(QPixmap::fromImage(qimage_.scaled(ui->label_camera->size(), Qt::KeepAspectRatio)));
    qimage_mutex_.unlock();
}

void MainWindow::displayText(const QString &line_error_string, const QString &height_string) {
  ui->leadingLine_param->setText(line_error_string);
  ui->height_value->setText(height_string);
}

void MainWindow::on_pushButton_clicked() {
    auto b = new BigCamera;
    b->show();
    //      this->close();
}

// 尝试自编延时函数
// void Sleep(int msec)
// {
//   QTime dieTime = QTime::currentTime().addMSecs(msec);
//   while( QTime::currentTime() < dieTime )
//     QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
// }

void MainWindow::on_horizontalSlider_sliderMoved(int position) {
    QString text = QString::number(30 * position);
    this->ui->lineEdit_2->setText(text);
}

void MainWindow::on_pushButton_5_clicked() {
    carspeed = carspeed - 0.25;
    float speed = this->ui->lineEdit_13->text().toDouble();
    speed = carspeed;
    QString qspeed = QString::number(speed);
    this->ui->lineEdit_13->setText(qspeed);
    qnode.pub_car_speed(speed);
}

void MainWindow::on_pushButton_13_clicked() {
    carspeed = carspeed + 0.25;
    float speed = this->ui->lineEdit_13->text().toDouble();
    speed = carspeed;
    QString qspeed = QString::number(speed);
    this->ui->lineEdit_13->setText(qspeed);
    qnode.pub_car_speed(speed);
}

void MainWindow::on_stop_button_clicked() {
    int is_stop = 1;
    ui->pushButton_10->setStyleSheet("background-color: green");
    this->ui->stop_button->setStyleSheet("background-color: red");
    qnode.pub_is_stop(is_stop);
}

void MainWindow::on_ublock_button_clicked() {
    bool is_stop = false;
    this->ui->stop_button->setStyleSheet("background-color: white");
    qnode.pub_is_stop(is_stop);
}


void MainWindow::on_go_left_button_clicked()
{
  position = position +1000;
  int turn = position;
  QString qturn = QString::number(turn);
  this->ui->lineEdit_13->setText(qturn);
  qnode.pub_car_turn(turn);
  std::cout<<"left_mainwindows"<<std::endl;
}

void MainWindow::on_go_right_button_clicked()
{
  position = position -1000;
  int turn = position;
  qnode.pub_car_turn(turn);

}

void MainWindow::on_height_button_clicked()
{
    qnode.pub_height_control_mode(120);
    usleep(500000);
    qnode.pub_height_control_mode(100);
}

void MainWindow::on_down_button_clicked()
{
    qnode.pub_height_control_mode(110);
    usleep(500000);
    qnode.pub_height_control_mode(100);
}

void MainWindow::on_height_button_2_clicked()
{
    qnode.pub_height_control_mode(102);
    usleep(500000);
    qnode.pub_height_control_mode(100);
}

void MainWindow::on_down_button_2_clicked()
{
    qnode.pub_height_control_mode(101);
    usleep(500000);
    qnode.pub_height_control_mode(100);
}

void MainWindow::on_pushButton_8_clicked()
{
    qnode.pub_height_control_mode(100);
}
