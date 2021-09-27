#include "controller.h"
#include "ui_controller.h"


Controller::Controller(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Controller), UDPsend()
{
    ui->setupUi(this);
    qthread = new qTh(this);
    qRegisterMetaType<UDPsock::UDP_DATA>("UDPsock::UDP_DATA");
    connect(qthread, SIGNAL(setValue(UDPsock::UDP_DATA)),this,SLOT(UDPrecvDATA(UDPsock::UDP_DATA)));
    qthread->start();

    /* Setup Multicast UDP send&recv */
    UDPsend.GROUP_ = "239.255.255.250";
    UDPsend.PORT_ = 9307;
    UDPsend.sendInit();

    int DefaultVel = 0; // cm/s
    MinVel = 0; // cm/s
    MaxVel = 140; // cm/s
    int DefaultDist = 80; // cm
    MinDist = 10; // cm
    MaxDist = 200; // cm

    FV1_cf = 0;
    FV2_cf = 0;

    /* Setup Velocity Slider */
    ui->MVelSlider->setMaximum(MaxVel);
    ui->LVVelSlider->setMaximum(MaxVel);
    ui->FV1VelSlider->setMaximum(MaxVel);
    ui->FV2VelSlider->setMaximum(MaxVel);
    ui->MVelSlider->setMinimum(MinVel);
    ui->LVVelSlider->setMinimum(MinVel);
    ui->FV1VelSlider->setMinimum(MinVel);
    ui->FV2VelSlider->setMinimum(MinVel);
    ui->MVelSlider->setValue(DefaultVel);
    ui->LVVelSlider->setValue(DefaultVel);
    ui->FV1VelSlider->setValue(DefaultVel);
    ui->FV2VelSlider->setValue(DefaultVel);

    /* Setup Distance Slider */
    ui->MDistSlider->setMaximum(MaxDist);
    ui->LVDistSlider->setMaximum(MaxDist);
    ui->FV1DistSlider->setMaximum(MaxDist);
    ui->FV2DistSlider->setMaximum(MaxDist);
    ui->MDistSlider->setMinimum(MinDist);
    ui->LVDistSlider->setMinimum(MinDist);
    ui->FV1DistSlider->setMinimum(MinDist);
    ui->FV2DistSlider->setMinimum(MinDist);
    ui->MDistSlider->setValue(DefaultDist);
    ui->LVDistSlider->setValue(DefaultDist);
    ui->FV1DistSlider->setValue(DefaultDist);
    ui->FV2DistSlider->setValue(DefaultDist);

    /* Setup Distance Bar */
    ui->LVVelBar->setMaximum(MaxVel);
    ui->FV1VelBar->setMaximum(MaxVel);
    ui->FV2VelBar->setMaximum(MaxVel);
    ui->LVVelBar->setMinimum(MinVel);
    ui->FV1VelBar->setMinimum(MinVel);
    ui->FV2VelBar->setMinimum(MinVel);
    ui->LVDistBar->setMaximum(MaxDist);
    ui->FV1DistBar->setMaximum(MaxDist);
    ui->FV2DistBar->setMaximum(MaxDist);
    ui->LVDistBar->setMinimum(MinDist);
    ui->FV1DistBar->setMinimum(MinDist);
    ui->FV2DistBar->setMinimum(MinDist);

    /* Setup Mode */
    ui->LVBox->setCurrentIndex(1);
    ui->FV1Box->setCurrentIndex(2);
    ui->FV2Box->setCurrentIndex(3);

    UDPsendDATA(DefaultVel, DefaultDist, 0);
    UDPsendDATA(DefaultVel, DefaultDist, 1);
    UDPsendDATA(DefaultVel, DefaultDist, 2);
}

void Controller::UDPsendDATA(int value_vel, int value_dist, int to)
{
    UDPsock::UDP_DATA udpData;
    udpData.index = 307;
    udpData.to = to;
    udpData.cf = 0;
    if(to > 0){
        if(to == 1)
            udpData.cf = FV1_cf;
        if(to == 2)
            udpData.cf = FV2_cf;
    }

    if(value_vel >= 10) {
      udpData.target_vel = value_vel/100.0;
      udpData.sync = 1;
    }
    else {
      udpData.target_vel = 0;
      udpData.sync = 0;
    }
    udpData.current_vel = 0;
    udpData.target_dist = value_dist/100.0;
    udpData.current_dist = 0;
    UDPsend.sendData(udpData);
}

void Controller::UDPrecvDATA(UDPsock::UDP_DATA value)
{
    UDPsock::UDP_DATA tmp;
    int vel, dist;
    float deci = 100;
    tmp = value;
    vel = tmp.current_vel*deci;
    dist = tmp.current_dist*deci;
    if(tmp.index == 0)
    {
        ui->LVCurVel->setText(QString::number(vel/deci));
        if(vel > MaxVel) {
            vel = MaxVel;
        }
        ui->LVVelBar->setValue(vel);
        ui->LVCurDist->setText(QString::number(dist/deci));
        if(dist > MaxDist) {
            dist = MaxDist;
        }
        ui->LVDistBar->setValue(dist);
        cv::Mat frame;
        display_Map(value).copyTo(frame);
        ui->LV_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));
    } else if (value.index == 1)
    {
        ui->FV1CurVel->setText(QString::number(vel/deci));
        if(vel > MaxVel) {
            vel = MaxVel;
        }
        ui->FV1VelBar->setValue(vel);
        ui->FV1CurDist->setText(QString::number(dist/deci));
        if(dist > MaxDist) {
            dist = MaxDist;
        }
        ui->FV1DistBar->setValue(dist);
        cv::Mat frame;
        display_Map(value).copyTo(frame);
        ui->FV1_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));
    } else if (value.index == 2)
    {
        ui->FV2CurVel->setText(QString::number(vel/deci));
        if(vel > MaxVel) {
            vel = MaxVel;
        }
        ui->FV2VelBar->setValue(vel);
        ui->FV2CurDist->setText(QString::number(dist/deci));
        if(dist > MaxDist) {
            dist = MaxDist;
        }
        ui->FV2DistBar->setValue(dist);
        cv::Mat frame;
        display_Map(value).copyTo(frame);
        ui->FV2_MAP->setPixmap(QPixmap::fromImage(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888)));
    }
}

cv::Mat Controller::display_Map(UDPsock::UDP_DATA value)
{
    int width = 350;
    int height = 350;
    cv::Mat map_frame = cv::Mat::zeros(cv::Size(width,height), CV_8UC3);
    int check_dist = 50;
    for(int i=0;i<height;i+=check_dist)
        cv::line(map_frame, cv::Point(0,i), cv::Point(width,i),cv::Scalar::all(100));
    for(int i=0;i<width;i+=check_dist)
        cv::line(map_frame, cv::Point(i,0), cv::Point(i,height),cv::Scalar::all(100));

    int centerY = height-100, centerX=width/2;
    cv::rectangle(map_frame, cv::Rect(cv::Point(centerX-9, centerY+50), cv::Point(centerX+9, centerY)), cv::Scalar(20,20,255), -1);

    std::vector<cv::Point> RpointList;
    std::vector<cv::Point> LpointList;
    std::vector<cv::Point> CpointList;
    int cam_height = 480, cam_width = 640; // pixel
    float cam_y_dist = 98, cam_x_dist = 77; // cm
    for(int i = -550; i < cam_height*1.2; i++){
        cv::Point temp_point;
        temp_point.y = i*cam_y_dist/cam_height-26;
        temp_point.y += (centerY-cam_y_dist);
        temp_point.x = (value.coef[0].a*pow(i,2) + value.coef[0].b*i + value.coef[0].c)*cam_x_dist/cam_width;
        temp_point.x += centerX-cam_x_dist/2;
        RpointList.push_back(temp_point);
        temp_point.x = (value.coef[1].a*pow(i,2) + value.coef[1].b*i + value.coef[1].c)*cam_x_dist/cam_width;
        temp_point.x += centerX-cam_x_dist/2;
        LpointList.push_back(temp_point);
        temp_point.x = (value.coef[2].a*pow(i,2) + value.coef[2].b*i + value.coef[2].c)*cam_x_dist/cam_width;
        temp_point.x += centerX-cam_x_dist/2;
        CpointList.push_back(temp_point);
    }
    const cv::Point* right_points_point = (const cv::Point*) cv::Mat(RpointList).data;
    int right_points_number = cv::Mat(RpointList).rows;
    const cv::Point* left_points_point = (const cv::Point*) cv::Mat(LpointList).data;
    int left_points_number = cv::Mat(LpointList).rows;
    const cv::Point* center_points_point = (const cv::Point*) cv::Mat(CpointList).data;
    int center_points_number = cv::Mat(CpointList).rows;

    cv::polylines(map_frame, &right_points_point, &right_points_number, 1, false, cv::Scalar::all(255), 2);
    cv::polylines(map_frame, &left_points_point, &left_points_number, 1, false, cv::Scalar::all(255), 2);
    cv::polylines(map_frame, &center_points_point, &center_points_number, 1, false, cv::Scalar(200,255,200), 2);

    float r = value.current_dist*100; // cm
    float theta = value.current_angle*M_PI/180; // rad
    if(r < 250) {
      int X = r*sin(theta)+centerX;
      int Y = -r*cos(theta)+centerY;
      cv::circle(map_frame,cv::Point(X,Y), 5, cv::Scalar(50,50,255), -1);
    }

    cv::line(map_frame, cv::Point(width/2 - check_dist,124 + value.roi_dist*100/490),cv::Point(width/2 + check_dist,124+value.roi_dist*100/490), cv::Scalar(255,100,100),3);

    cv::Mat swap_frame;
    cv::cvtColor(map_frame, swap_frame, cv::COLOR_BGR2RGB);
    return swap_frame;
}

Controller::~Controller()
{
    delete ui;
}


void Controller::on_MVelSlider_valueChanged(int value)
{
    ui->MTarVel->setText(QString::number(value/100.0)); // m/s
    ui->LVVelSlider->setValue(value);
    ui->FV1VelSlider->setValue(value);
    ui->FV2VelSlider->setValue(value);
}

void Controller::on_MDistSlider_valueChanged(int value)
{
    ui->MTarDist->setText(QString::number(value/100.0)); // m
    ui->LVDistSlider->setValue(value);
    ui->FV1DistSlider->setValue(value);
    ui->FV2DistSlider->setValue(value);
}

void Controller::on_LVVelSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    value_vel = value;
    value_dist = ui->LVDistSlider->value();
    UDPsendDATA(value_vel, value_dist, 0);

    ui->LVTarVel->setText(QString::number(value/100.0)); // m/s
}

void Controller::on_LVDistSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    value_vel = ui->LVVelSlider->value();
    value_dist = value;
    UDPsendDATA(value_vel, value_dist, 0);

    ui->LVTarDist->setText(QString::number(value/100.0)); // m
}

void Controller::on_FV1VelSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    value_vel = value;
    value_dist = ui->FV1DistSlider->value();
    UDPsendDATA(value_vel, value_dist, 1);

    ui->FV1TarVel->setText(QString::number(value/100.0)); // m/s
}

void Controller::on_FV1DistSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    value_vel = ui->FV1VelSlider->value();
    value_dist = value;
    UDPsendDATA(value_vel, value_dist, 1);

    ui->FV1TarDist->setText(QString::number(value/100.0)); // m
}

void Controller::on_FV2VelSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    value_vel = value;
    value_dist = ui->FV2DistSlider->value();
    UDPsendDATA(value_vel, value_dist, 2);

    ui->FV2TarVel->setText(QString::number(value/100.0)); // m/s
}

void Controller::on_FV2DistSlider_valueChanged(int value)
{
    int value_vel, value_dist;
    value_vel = ui->FV2VelSlider->value();
    value_dist = value;
    UDPsendDATA(value_vel, value_dist, 2);

    ui->FV2TarDist->setText(QString::number(value/100.0)); // m
}

void Controller::on_pushButton_clicked()
{
    int LV_dist = ui->LVDistSlider->value();
    int FV1_dist = ui->FV1DistSlider->value();
    int FV2_dist = ui->FV2DistSlider->value();
    ui->MVelSlider->setValue(0);
    ui->LVVelSlider->setValue(0);
    ui->FV1VelSlider->setValue(0);
    ui->FV2VelSlider->setValue(0);
    UDPsendDATA(0, LV_dist, 0);
    UDPsendDATA(0, FV1_dist, 1);
    UDPsendDATA(0, FV2_dist, 2);
}

void Controller::on_LVBox_activated(int index)
{
    if(index == 0)
    {
        ui->LVVelSlider->setEnabled(true);
        ui->LVDistSlider->setEnabled(true);
    }
    else if(index == 1)
    {
        ui->LVVelSlider->setEnabled(true);
        ui->LVDistSlider->setEnabled(true);
    }
    else if(index == 2)
    {
        ui->LVVelSlider->setEnabled(false);
        ui->LVDistSlider->setEnabled(true);
    }
    else if(index == 3)
    {
        ui->LVVelSlider->setEnabled(false);
        ui->LVDistSlider->setEnabled(true);
    }
}


void Controller::on_FV1Box_activated(int index)
{
    if(index == 0)
    {
        ui->FV1VelSlider->setEnabled(true);
        ui->FV1DistSlider->setEnabled(true);
    }
    else if(index == 1)
    {
        ui->FV1VelSlider->setEnabled(true);
        ui->FV1DistSlider->setEnabled(true);
    }
    else if(index == 2)
    {
        ui->FV1VelSlider->setEnabled(false);
        ui->FV1DistSlider->setEnabled(true);
    }
    else if(index == 3)
    {
        ui->FV1VelSlider->setEnabled(false);
        ui->FV1DistSlider->setEnabled(true);
    }
}


void Controller::on_FV2Box_activated(int index)
{
    if(index == 0)
    {
        ui->FV2VelSlider->setEnabled(true);
        ui->FV2DistSlider->setEnabled(true);
    }
    else if(index == 1)
    {
        ui->FV2VelSlider->setEnabled(true);
        ui->FV2DistSlider->setEnabled(true);
    }
    else if(index == 2)
    {
        ui->FV2VelSlider->setEnabled(false);
        ui->FV2DistSlider->setEnabled(true);
    }
    else if(index == 3)
    {
        ui->FV2VelSlider->setEnabled(false);
        ui->FV2DistSlider->setEnabled(true);
    }
}

void Controller::on_Send_clicked()
{
    int vel = ui->MTarVel->text().split(" ")[0].toFloat()*100;
    ui->MVelSlider->setValue(vel);
    ui->LVVelSlider->setValue(vel);
    ui->FV1VelSlider->setValue(vel);
    ui->FV2VelSlider->setValue(vel);
    int dist = ui->MTarDist->text().split(" ")[0].toFloat()*100;
    ui->MDistSlider->setValue(dist);
    ui->LVDistSlider->setValue(dist);
    ui->FV1DistSlider->setValue(dist);
    ui->FV2DistSlider->setValue(dist);
    UDPsendDATA(ui->LVVelSlider->value(), ui->LVDistSlider->value(), 0);
    //UDPsendDATA(ui->FV1VelSlider->value(), ui->FV1DistSlider->value(), 1);
    //UDPsendDATA(ui->FV2VelSlider->value(), ui->FV2DistSlider->value(), 2);
}


void Controller::on_FV1_cf_toggled(bool checked)
{
    int value_vel = ui->FV1VelSlider->value();
    int value_dist = ui->FV1DistSlider->value();
    FV1_cf = checked;
    UDPsendDATA(value_vel, value_dist, 1);
}


void Controller::on_FV2_cf_toggled(bool checked)
{
    int value_vel = ui->FV2VelSlider->value();
    int value_dist = ui->FV2DistSlider->value();
    FV2_cf = checked;
    UDPsendDATA(value_vel, value_dist, 2);
}
