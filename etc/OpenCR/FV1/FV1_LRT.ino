#include <stdio.h>
#include <Servo.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <SD.h>
#include <IMU.h>
#include <ctl.h>
#include <vel.h>

// Period
#define BAUD_RATE     (57600)
#define CYCLE_TIME    (100000) // us
#define SEC_TIME      (1000000) // us
#define T_TIME        (100) // us
#define ANGLE_TIME    (33000) // us

// PIN
#define STEER_PIN     (6)
#define SD_PIN        (10)
#define THROTTLE_PIN  (9)
#define EN_PINA       (3)
#define EN_PINB       (2)

// Encoder
#define TICK2CYCLE    (60) // (65) // 65 ticks(EN_pos_) = 1 wheel cycle
#define WHEEL_DIM     (0.085) // m
#define MAX_SPEED     (2)  // m/s
#define MAX_PWM       (2000)
#define MIN_PWM       (1600)
#define ZERO_PWM      (1500)
#define MAX_STEER     (1800)
#define MIN_STEER     (1200)
#define STEER_CENTER  (1480)

#define DATA_LOG      (0)

cIMU  IMU;
Servo throttle_;
Servo steer_;
int Index_;
float raw_throttle_;
float tx_throttle_;
float tx_steer_;
float tx_dist_;
float tx_tdist_;
float output_;
float crc_vel_;
volatile int EN_pos_;
volatile int CountT_;
volatile int cumCountT_;
char filename_[] = "LV1_00.TXT";
File logfile_;

HardwareTimer Timer1(TIMER_CH1); // T Method
HardwareTimer Timer2(TIMER_CH2); // Check EN
HardwareTimer Timer3(TIMER_CH3); // Angle

/*
   ros Subscribe Callback Function
*/
void rosCtlCallback(const scale_truck_control::ctl& msg) {
  Index_ = msg.index;
  tx_throttle_ = msg.send_vel;
  tx_tdist_ = msg.ref_dist;
  tx_dist_ = msg.cur_dist;
  tx_steer_ = msg.steer_angle;  // float32
}
void rosLrcCallback(const scale_truck_control::lrc& msg) {
  crc_vel_ = msg.crc_vel;
}
/*
   SPEED to RPM
*/
float Kp_dist_ = 1.0; // 2.0; //0.8;
float Kd_dist_ = 0.05; //0.05;
float Kp_ = 0.8; // 2.0; //0.8;
float Ki_ = 2.0; // 0.4; //10.0;
float Kd_ = 0.0; //0.05;
float Ka_ = 0.01;
float Kf_ = 1.0;  // feed forward const.
float dt_ = 0.1;
float circ_ = WHEEL_DIM * M_PI;
bool alpha_ = false;
float tmp_ = 0.0;
float epsilon_ = 0.5;
scale_truck_control::vel vel_msg_;
scale_truck_control::lrc lrc_msg_;
sensor_msgs::Imu imu_msg_;
float setSPEED(float tar_vel, float cur_vel) { 
  static float output, err, prev_err, P_err, I_err, D_err;
  static float prev_u_k, prev_u, A_err;
  static float dist_err, prev_dist_err, P_dist_err, D_dist_err;
  static float lo_vel = 0.0;
  static float A = 0.6817;
  static float B = 0.3183;
  static float L = 0.2817;
  float u, u_k;
  float u_dist, u_dist_k;
  float ref_vel;
  vel_msg_.cur_vel = cur_vel;
  if(tar_vel <= 0 ) {
    output = ZERO_PWM;
    I_err = 0;
    A_err = 0;
    vel_msg_.ref_vel = 0;
  } else {
    if(Index_!=0) {
      dist_err = tx_dist_ - tx_tdist_;    
      P_dist_err = Kp_dist_ * dist_err;
      D_dist_err = (Kd_dist_ * ((dist_err - prev_dist_err) / dt_ )); 
      u_dist = P_dist_err + D_dist_err + tar_vel;
  
      // sat(u(k))  saturation start 
      if(u_dist > 1.2) u_dist_k = 1.2;
      else if(u_dist <= 0) u_dist_k = 0;
      else u_dist_k = u_dist;
      
      ref_vel = u_dist_k;
    } else {
      ref_vel = tar_vel;
    }
    
    vel_msg_.ref_vel = ref_vel;
    
    err = ref_vel - cur_vel;
    P_err = Kp_ * err;
    I_err += Ki_ * err * dt_;
    A_err += Ka_ * ((prev_u_k - prev_u) / dt_);
    u = P_err + I_err + A_err + ref_vel * Kf_;

    if(u > 2.0) u_k = 2.0;
    else if(u <= 0) u_k = 0;
    else u_k = u;

    // inverse function 
    output = (-8.152e-02 + sqrt(pow(-8.152e-02,2)-4*(-2.0975e-05)*(-76.87-u_k)))/(2*(-2.0975e-05));
    //output = tx_throttle_;
	
  }
  // output command
  lo_vel = A * lo_vel + B * u_k + L * (cur_vel - lo_vel);
  tmp_ = cur_vel - lo_vel;
  if(fabs(cur_vel - lo_vel) > epsilon_){
	  alpha_ = true;
  }
  else{
	  alpha_ = false;
  }
  lrc_msg_.alpha = alpha_;
  prev_u_k = u_k;
  prev_u = u;
  prev_dist_err = dist_err;
  throttle_.writeMicroseconds(output);
  return output;
}
/*
   ANGLE to PWM
*/
void setANGLE() {
  static float output;
  float angle = tx_steer_;
  if(IMU.update() > 0) {
    imu_msg_.orientation.x = IMU.quat[0];
    imu_msg_.orientation.y = IMU.quat[1];
    imu_msg_.orientation.z = IMU.quat[2];
    imu_msg_.orientation.w = IMU.quat[3];
    imu_msg_.angular_velocity.x = IMU.angle[0];
    imu_msg_.angular_velocity.y = IMU.angle[1];
    imu_msg_.angular_velocity.z = IMU.angle[2];
    imu_msg_.linear_acceleration.x = IMU.rpy[0];
    imu_msg_.linear_acceleration.y = IMU.rpy[1];
    imu_msg_.linear_acceleration.z = IMU.rpy[2];
  }
  output = (angle * 12.0) + (float)STEER_CENTER;
  if(output > MAX_STEER)
    output = MAX_STEER;
  else if(output < MIN_STEER)
    output = MIN_STEER;
  steer_.writeMicroseconds(output);
}
/*
   Encoder A interrupt service routine
*/
void getENA() {
  if (digitalRead(EN_PINA) == HIGH) {
    if (digitalRead(EN_PINB) == LOW) {
      EN_pos_ += 1;  // white + black
    }
    else {
      EN_pos_ -= 1; // white - black
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    if (digitalRead(EN_PINB) == HIGH) {
      EN_pos_ += 1;
    }
    else {
      EN_pos_ -= 1;
    }
  }
  cumCountT_ += CountT_;
  CountT_ = 0;
}
/*
   RPM Check Function
*/
void CheckEN() {
  static float output_vel;
  static float output_angle;
  static float cur_vel;
  static float target_vel;
  static float target_ANGLE;
  static float target_RPM;
  static float cur_RPM;
  target_vel = tx_throttle_; // m/s
  target_ANGLE = tx_steer_; // degree
  if(cumCountT_ == 0)
    cur_vel = 0;
  else
     cur_vel = (float)EN_pos_ / TICK2CYCLE * ( SEC_TIME / ((float)cumCountT_*T_TIME)) * circ_; // m/s
  output_vel = setSPEED(target_vel, cur_vel);
  output_angle = IMU.rpy[2];
  if(DATA_LOG)
  {
    Serial.print(target_vel);
    Serial.print(" m/s | ");
    Serial.print(cur_vel);
    Serial.print(" m/s | ");
    Serial.print(output_vel);
    Serial.println(" signal | ");
    Serial.print(EN_pos_);
    Serial.print(" count | ");
    Serial.print(cumCountT_);
    Serial.print(" count | ");
    Serial.print(output_vel);
    Serial.print(" us | ");
    Serial.print(target_ANGLE);
    Serial.print(" deg | ");
    Serial.print(output_angle);
    Serial.println(" deg");
  }
  logfile_ = SD.open(filename_, FILE_WRITE);
  logfile_.print(target_vel);
  logfile_.print(",");
  logfile_.print(cur_vel);
  logfile_.print(",");
  logfile_.print(EN_pos_);
  logfile_.print(",");
  logfile_.print(cumCountT_);
  logfile_.print(",");
  logfile_.print(output_vel);
  logfile_.print(",");
  logfile_.print(Kp_);
  logfile_.print(",");
  logfile_.print(Ki_);
  logfile_.print(",");
  logfile_.print(Kd_);
  logfile_.print(",");
  logfile_.print(tx_dist_);
  logfile_.print(",");
  logfile_.print(target_ANGLE);
  logfile_.print(",");
  logfile_.print(IMU.rpy[0]);
  logfile_.print(",");
  logfile_.print(IMU.rpy[1]);
  logfile_.print(",");
  logfile_.println(IMU.rpy[2]);
  logfile_.close();
  // CLEAR counter
  ClearT();
}
void ClearT() {
  EN_pos_ = 0;
  cumCountT_ = 0;
}
void CountT() {
  CountT_ += 1;
}
/*
   ros variable
*/
ros::NodeHandle nh_;
ros::Subscriber<scale_truck_control::ctl> rosSubMsg("/ctl_msg", &rosCtlCallback);
ros::Subscriber<scale_truck_control::lrc> rosSubLrc("/lrc_msg", &rosLrcCallback);
ros::Publisher rosPubVel("/vel_msg", &vel_msg_);
ros::Publisher rosPubImu("/imu_msg", &imu_msg_);
ros::Publisher rosPubLrc("/lrc_msg", &lrc_msg_);
/*
   Arduino setup()
*/
void setup() {
  nh_.initNode();
  nh_.subscribe(rosSubMsg);
  nh_.subscribe(rosSubLrc);
  nh_.advertise(rosPubVel);
  nh_.advertise(rosPubImu);
  nh_.advertise(rosPubLrc);
  throttle_.attach(THROTTLE_PIN);
  steer_.attach(STEER_PIN);
  pinMode(EN_PINA, INPUT);
  pinMode(EN_PINB, INPUT);
  attachInterrupt(0, getENA, CHANGE);
  IMU.begin();
  Serial.begin(BAUD_RATE);
  if(!SD.begin(10)){
    Serial.println("Card failed, or not present");
  } else {
    Serial.println("card initialized.");
    for(uint8_t i=0; i<100; i++){
      filename_[4] = i/10 + '0';
      filename_[5] = i%10 + '0';
      if(! SD.exists(filename_)){
        logfile_ = SD.open(filename_, FILE_WRITE);
        break;
      }
    }
    Serial.print("Logging to: ");
    Serial.print(filename_);
    logfile_.close();
  }
  Timer1.stop();
  Timer1.setPeriod(T_TIME);
  Timer1.attachInterrupt(CountT);
  Timer1.start();
  Timer2.stop();
  Timer2.setPeriod(CYCLE_TIME);
  Timer2.attachInterrupt(CheckEN);
  Timer2.start();
  Timer3.stop();
  Timer3.setPeriod(ANGLE_TIME);
  Timer3.attachInterrupt(setANGLE);
  Timer3.start();
  Serial.print("[OpenCR] setup()");
  tx_throttle_ = 0.0;
  tx_steer_ = 0.0;
}
/*
   Arduino loop()
*/
void loop() {
  static unsigned long prevTime = 0;
  static unsigned long currentTime;
  
  if(DATA_LOG)
  {
    static float speed_vel;
    static boolean flag_ = false;
    if(Serial.available() > 0) {
      //tx_steer_ = Serial.parseFloat();
      //tx_throttle_ = Serial.parseFloat();
      speed_vel = Serial.parseFloat();
      flag_ = true;
      Serial.println(speed_vel);
    }
    if(flag_) {
      delay(1000);
      tx_throttle_ = speed_vel;
      delay(5000);
      tx_throttle_ = 0;
      flag_ = false;
    }
  }
  
  nh_.spinOnce();
  delay(1);
  
  currentTime = millis();
  if ((currentTime - prevTime) >= (ANGLE_TIME / 1000)) {
    rosPubVel.publish(&vel_msg_);
    rosPubImu.publish(&imu_msg_);
	rosPubLrc.publish(&lrc_msg_);
    prevTime = currentTime;
  }
}