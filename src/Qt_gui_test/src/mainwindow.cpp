#include "mainwindow.h"
#include "bigcamera.h"
#include "ui_mainwindow.h"
#include <Widget.h>
#include <qnamespace.h>

MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
	QMainWindow(parent),
	ui(new Ui::MainWindow),
	qnode(argc,argv)
{
	ui->setupUi(this);

	// Widget paint_line;
	// paint_line.show();
	// 实际上这里不需要做任何的事情，因为promote的widget出来就是自己显示的。
	ui->widget->show();
	// 在主函数里仅仅可以强制隐藏。
	// ui->widget->hide();
	// displayChart();

	// QProgressBar *m_pLeftToRightProBar = new QProgressBar(this);
	// m_pLeftToRightProBar->setOrientation(Qt::Horizontal);  // 水平方向
	// m_pLeftToRightProBar->setMinimum(0);  // 最小值
	// m_pLeftToRightProBar->setMaximum(100);  // 最大值
	// m_pLeftToRightProBar->setValue(50);  // 当前进度

  std::cout<<"init ros node..."<<std::endl;
  qnode.init();

  ui->progressBar->setMinimum(18);
  ui->progressBar->setMaximum(120);
  ui->progressBar->setValue(18);

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

	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(&qnode,SIGNAL(loggingCamera()),this,SLOT(updateLogcamera()));
	QObject::connect(&qnode,SIGNAL(logging_leader_line_error()),this,SLOT(updateText()));
	QObject::connect(&qnode,SIGNAL(loggingChart()),this,SLOT(displayChart()));
	QObject::connect(&qnode,SIGNAL(logging_REEL_speed()),this,SLOT(updateREEL()));
  QObject::connect(&qnode,SIGNAL(logging_is_obstacle()),this,SLOT(update_is_obstacle()));
  QObject::connect(&qnode,SIGNAL(logging_no_obstacle()),this,SLOT(update_no_obstacle()));
  QObject::connect(&qnode,SIGNAL(logging_reap_height()),this,SLOT(update_reap_height()));
	// QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::on_pushButtonConnect_clicked()
{
  std::cout<<"init button pressed."<<std::endl;
    qnode.init();
}

void MainWindow::on_pushButtonSend_clicked()
{
	qnode.ros_test("I changed this world!");
}

void MainWindow::updateLogcamera()
{
	displayMat(qnode.image);
}

void MainWindow::updateText()
{
  displayText(qnode.leader_line_error_string);
  if(qnode.leader_line_error < -2){
    ui->go_left_button->setStyleSheet("background-color: red");
    ui->go_right_button->setStyleSheet("background-color: white");
  }
  else if(qnode.leader_line_error > 2){
    ui->go_left_button->setStyleSheet("background-color: white");
    ui->go_right_button->setStyleSheet("background-color: red");
  }
else{
    ui->go_left_button->setStyleSheet("background-color: white");
    ui->go_right_button->setStyleSheet("background-color: white");
}
}

void MainWindow::displayChart()
{
	ui->widget->c = qnode.chart;
}

void MainWindow::updateREEL()
{
	ui->progressBar1_4->setValue(qnode.REEL_speed);

	// for(int i=1;i<qnode.REEL_speed;i++)
	// {
	//         ui->progressBar1_4->setValue(i);
	// }
}

void MainWindow::update_is_obstacle()
{
//  std::cout<<"obstacle!"<<std::endl;
  ui->pushButton_10->setStyleSheet("background-color: red");
}

void MainWindow::update_no_obstacle()
{
//  std::cout<<"obstacle!"<<std::endl;
  ui->pushButton_10->setStyleSheet("background-color: green");
}

void MainWindow::update_reap_height()
{
  QString qheight;
  qheight = QString::number(qnode.reap_height);
  ui->lineEdit_41->setText(qheight);
  ui->progressBar->setValue(qnode.reap_height);
}

void MainWindow::displayMat(const QImage &image)
{
	qimage_mutex_.lock();
	qimage_ = image.copy();
	ui->label_camera->setPixmap(QPixmap::fromImage(qimage_.scaled(ui->label_camera->size(),Qt::KeepAspectRatio)));
	qimage_mutex_.unlock();
}

void MainWindow::displayText(const QString &text)
{
	ui->leadingLine_param->setText(text);
}

void MainWindow::on_pushButton_clicked()
{
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

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
	QString text = QString::number(30*position);
	this->ui->lineEdit_2->setText(text);
}

void MainWindow::on_pushButton_5_clicked()
{
	double speed = this->ui->lineEdit_13->text().toDouble();
  speed = speed-0.1;
	QString qspeed = QString::number(speed);
	this->ui->lineEdit_13->setText(qspeed);
  qnode.pub_car_speed(speed);
}

void MainWindow::on_pushButton_13_clicked()
{
	double speed = this->ui->lineEdit_13->text().toDouble();
  speed = speed+0.1;
	QString qspeed = QString::number(speed);
	this->ui->lineEdit_13->setText(qspeed);
  qnode.pub_car_speed(speed);
}

void MainWindow::on_stop_button_clicked()
{
    bool is_stop = true;
    qnode.pub_is_stop(is_stop);
}
void MainWindow::on_ublock_button_clicked()
{
    bool is_stop = false;
    qnode.pub_is_stop(is_stop);
}
