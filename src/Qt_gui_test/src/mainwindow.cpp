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

    ui->progressBar1_8->setMinimum(0);
    ui->progressBar1_8->setMaximum(3000);
    ui->progressBar1_8->setValue(0);

    ui->progressBar1_9->setMinimum(0);
    ui->progressBar1_9->setMaximum(3000);
    ui->progressBar1_9->setValue(0);

    ui->progressBar1_10->setMinimum(0);
    ui->progressBar1_10->setMaximum(3000);
    ui->progressBar1_10->setValue(0);

    ui->progressBar1_11->setMinimum(0);
    ui->progressBar1_11->setMaximum(3000);
    ui->progressBar1_11->setValue(0);

    ui->progressBar1_12->setMinimum(0);
    ui->progressBar1_12->setMaximum(3000);
    ui->progressBar1_12->setValue(0);

    ui->progressBar1_13->setMinimum(0);
    ui->progressBar1_13->setMaximum(3000);
    ui->progressBar1_13->setValue(0);

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(loggingCamera()), this, SLOT(updateLogcamera()));
    QObject::connect(&qnode, SIGNAL(logging_leader_line_error()), this, SLOT(updateText()));
    QObject::connect(&qnode, SIGNAL(loggingChart()), this, SLOT(displayChart()));
    QObject::connect(&qnode, SIGNAL(logging_speed_3()), this, SLOT(update_s3()));
    QObject::connect(&qnode, SIGNAL(logging_speed_4()), this, SLOT(update_s4()));
    QObject::connect(&qnode, SIGNAL(logging_speed_2()), this, SLOT(update_s2()));
    QObject::connect(&qnode, SIGNAL(logging_speed_1()), this, SLOT(update_s1()));
    QObject::connect(&qnode, SIGNAL(logging_speed_5()), this, SLOT(update_s5()));
    QObject::connect(&qnode, SIGNAL(logging_speed_7()), this, SLOT(update_s7()));
    QObject::connect(&qnode, SIGNAL(logging_speed_8()), this, SLOT(update_s8()));
    QObject::connect(&qnode, SIGNAL(logging_speed_11()), this, SLOT(update_s11()));
    QObject::connect(&qnode, SIGNAL(logging_speed_9()), this, SLOT(update_s9()));
    QObject::connect(&qnode, SIGNAL(logging_speed_10()), this, SLOT(update_s10()));

    QObject::connect(&qnode, SIGNAL(logging_current_3()), this, SLOT(update_c3()));
    QObject::connect(&qnode, SIGNAL(logging_current_4()), this, SLOT(update_c4()));
    QObject::connect(&qnode, SIGNAL(logging_current_2()), this, SLOT(update_c2()));
    QObject::connect(&qnode, SIGNAL(logging_current_1()), this, SLOT(update_c1()));
    QObject::connect(&qnode, SIGNAL(logging_current_5()), this, SLOT(update_c5()));
    QObject::connect(&qnode, SIGNAL(logging_current_7()), this, SLOT(update_c7()));
    QObject::connect(&qnode, SIGNAL(logging_current_8()), this, SLOT(update_c8()));
    QObject::connect(&qnode, SIGNAL(logging_current_11()), this, SLOT(update_c11()));
    QObject::connect(&qnode, SIGNAL(logging_current_9()), this, SLOT(update_c9()));
    QObject::connect(&qnode, SIGNAL(logging_current_10()), this, SLOT(update_c10()));
    
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

void MainWindow::update_s3() {
    QString qstr;
    qstr = QString::number(qnode.speed3);
    ui->lineEdit_50->setText(qstr);
    ui->progressBar1_4->setValue(qnode.speed3);
}

void MainWindow::update_s4() {
    QString qstr;
    qstr = QString::number(qnode.speed4);
    ui->lineEdit_54->setText(qstr);
    ui->progressBar1_5->setValue(qnode.speed4);
}

void MainWindow::update_s2() {
    QString qstr;
    qstr = QString::number(qnode.speed2);
    ui->lineEdit_55->setText(qstr);
    ui->progressBar1_6->setValue(qnode.speed2);
}

void MainWindow::update_s1() {
    QString qstr;
    qstr = QString::number(qnode.speed1);
    ui->lineEdit_56->setText(qstr);
    ui->progressBar1_7->setValue(qnode.speed1);
}

void MainWindow::update_s5() {
    QString qstr;
    qstr = QString::number(qnode.speed5);
    ui->lineEdit_57->setText(qstr);
    ui->progressBar1_8->setValue(qnode.speed5);
}
void MainWindow::update_s7() {
    QString qstr;
    qstr = QString::number(qnode.speed7);
    ui->lineEdit_58->setText(qstr);
    ui->progressBar1_9->setValue(qnode.speed7);
}
void MainWindow::update_s8() {
    QString qstr;
    qstr = QString::number(qnode.speed8);
    ui->lineEdit_59->setText(qstr);
    ui->progressBar1_10->setValue(qnode.speed8);
}
void MainWindow::update_s11() {
    QString qstr;
    qstr = QString::number(qnode.speed11);
    ui->lineEdit_60->setText(qstr);
    ui->progressBar1_11->setValue(qnode.speed11);
}
void MainWindow::update_s9() {
    QString qstr;
    qstr = QString::number(qnode.speed9);
    ui->lineEdit_61->setText(qstr);
    ui->progressBar1_12->setValue(qnode.speed9);
}
void MainWindow::update_s10() {
    QString qstr;
    qstr = QString::number(qnode.speed10);
    ui->lineEdit_62->setText(qstr);
    ui->progressBar1_13->setValue(qnode.speed10);
}

//// update current
void MainWindow::update_c3() {
    QString qstr;
    qstr = QString::number(qnode.current3);
    ui->lineEdit_9->setText(qstr);
}

void MainWindow::update_c4() {
    QString qstr;
    qstr = QString::number(qnode.current4);
    ui->lineEdit_10->setText(qstr);
}

void MainWindow::update_c2() {
    QString qstr;
    qstr = QString::number(qnode.current2);
    ui->lineEdit_11->setText(qstr);
}

void MainWindow::update_c1() {
    QString qstr;
    qstr = QString::number(qnode.current1);
    ui->lineEdit_12->setText(qstr);
}

void MainWindow::update_c5() {
    QString qstr;
    qstr = QString::number(qnode.current5);
    ui->lineEdit_20->setText(qstr);
}

void MainWindow::update_c7() {
    QString qstr;
    qstr = QString::number(qnode.current7);
    ui->lineEdit_21->setText(qstr);
}

void MainWindow::update_c8() {
    QString qstr;
    qstr = QString::number(qnode.current8);
    ui->lineEdit_22->setText(qstr);
}

void MainWindow::update_c11() {
    QString qstr;
    qstr = QString::number(qnode.current11);
    ui->lineEdit_23->setText(qstr);
}

void MainWindow::update_c9() {
    QString qstr;
    qstr = QString::number(qnode.current9);
    ui->lineEdit_24->setText(qstr);
}

void MainWindow::update_c10() {
    QString qstr;
    qstr = QString::number(qnode.current10);
    ui->lineEdit_39->setText(qstr);
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
    carspeed = carspeed - 1000;
    float speed = this->ui->lineEdit_13->text().toDouble();
    speed = carspeed;
    QString qspeed = QString::number(speed);
    this->ui->lineEdit_13->setText(qspeed);
    qnode.pub_car_speed(speed);
}

void MainWindow::on_pushButton_13_clicked() {
    carspeed = carspeed + 1000;
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

void MainWindow::on_height_button_3_clicked()
{
    qnode.pub_hmi_ready(5);
}
