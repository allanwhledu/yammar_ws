#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qnode.h>
#include <QImage>
#include <QPixmap>
#include <QMutex>
#include <QTime>
#include <QString>
#include <MyDrawImage.h>


using namespace test_gui;

namespace Ui {
    class MainWindow;
}


class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = nullptr);

    ~MainWindow();

private slots:

    void on_pushButtonSend_clicked();

    void updateLogcamera();

    void updateText();

    void displayChart();

    void update_s3();
    void update_s4();
    void update_s2();
    void update_s1();
    void update_s5();
    void update_s7();
    void update_s8();
    void update_s11();
    void update_s9();
    void update_s10();

    void update_c3();
    void update_c4();
    void update_c2();
    void update_c1();
    void update_c5();
    void update_c7();
    void update_c8();
    void update_c11();
    void update_c9();
    void update_c10();

    void update_is_obstacle();

    void update_no_obstacle();

    void update_reap_height1();

    void update_reap_height2();

    void update_torque();

    void displayMat(const QImage &image);

    void displayText(const QString &line_error_string, const QString &height_string);

    void on_pushButton_clicked();

    void on_horizontalSlider_sliderMoved(int position);

    void on_pushButton_5_clicked();

    void on_pushButton_13_clicked();

    void on_pushButtonConnect_clicked();

    void on_stop_button_clicked();

    void on_ublock_button_clicked();

    void on_go_left_button_clicked();

    void on_go_right_button_clicked();

    void on_height_button_clicked();

    void on_down_button_clicked();

    void on_height_button_2_clicked();

    void on_down_button_2_clicked();

    void on_pushButton_8_clicked();

    void on_height_button_3_clicked();

    void on_height_button_4_clicked();

    void on_height_button_5_clicked();

private:
    Ui::MainWindow *ui;
    QNode qnode;
    QImage qimage_;
    mutable QMutex qimage_mutex_;
};

#endif // MAINWINDOW_H
