#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QMainWindow>
#include <sock_udp.hpp>
#include <qTh.h>

#include <opencv2/opencv.hpp>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui { class Controller; }
QT_END_NAMESPACE

class Controller : public QMainWindow
{
    Q_OBJECT

public:
    Controller(QWidget *parent = nullptr);
    ~Controller();
    void UDPsendDATA(int value_vel, int value_dist, int to);

    int MinVel;
    int MaxVel;
    int MinDist;
    int MaxDist;
    int FV1_cf;
    int FV2_cf;

private slots:
    void on_MVelSlider_valueChanged(int value);

    void on_MDistSlider_valueChanged(int value);

    void on_LVVelSlider_valueChanged(int value);

    void on_LVDistSlider_valueChanged(int value);

    void on_FV1VelSlider_valueChanged(int value);

    void on_FV1DistSlider_valueChanged(int value);

    void on_FV2VelSlider_valueChanged(int value);

    void on_FV2DistSlider_valueChanged(int value);

    void on_pushButton_clicked();

    void UDPrecvDATA(UDPsock::UDP_DATA value);
    cv::Mat display_Map(UDPsock::UDP_DATA value);

    void on_LVBox_activated(int index);

    void on_FV1Box_activated(int index);

    void on_FV2Box_activated(int index);

    void on_Send_clicked();

    void on_FV1_cf_toggled(bool checked);

    void on_FV2_cf_toggled(bool checked);

private:
    Ui::Controller *ui;
    UDPsock::UDPsocket UDPsend;
    qTh* qthread;
};
#endif // CONTROLLER_H
