#include <stdio.h>
#include <Servo.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <SD.h>
#include <IMU.h>
#define BAUD_RATE     (57600)
#define TICK2CYCLE    (60) // (65) // 65 ticks(EN_pos_) = 1 wheel cycle
#define CYCLE_TIME    (100000) // us 주기 수정
#define SEC_TIME      (1000000) // us
#define T_TIME        (100) // us
#define ANGLE_TIME    (100000) // us
// PIN
#define STEER_PIN     (6)
#define SD_PIN        (10)
#define THROTTLE_PIN  (9)
#define EN_PINA       (3)
#define EN_PINB       (2)
#define WHEEL_DIM     (0.085) // m
#define MAX_SPEED     (4)  // m/s
#define MAX_PWM       (2000)
#define MIN_PWM       (1600)
#define ZERO_PWM      (1580)
#define MAX_STEER     (1800)
#define MIN_STEER     (1200)
#define STEER_CENTER  (1480)
/*
   PWM variable
*/
cIMU  IMU;
Servo throttle_;
Servo steer_;
float tx_throttle_;
float test_throttle_;
float tx_steer_;
float tx_dist_;
float tx_tdist_;
float output_;
volatile int EN_pos_;
volatile int CountT_;
volatile int cumCountT_;
char filename_[] = "LOG00.TXT";
File logfile_;
HardwareTimer Timer1(TIMER_CH1); // T Method
HardwareTimer Timer2(TIMER_CH2); // Check EN
HardwareTimer Timer3(TIMER_CH3); // Angle
/*
   ros Subscribe Callback Function
*/
void rosTwistCallback(const geometry_msgs::Twist& msg) {
  tx_throttle_ = msg.linear.x;
  tx_tdist_ = msg.linear.z;
  tx_dist_ = msg.linear.y;
  tx_steer_ = msg.angular.z;
}
/*
   SPEED to RPM
*/
float prev_err_ = 0;
float result_;
float prev_u_k = 0;
float prev_u = 0;
float P_err = 0;
float I_err = 0;
float A_err = 0;
float D_err = 0;
float Kp_ = 0.5; // Kp_  0.5 
float Ki_ = 0.5;
float Kd_ = 0.001; // Kd_ = 0.1 ,0.01 , 0.001;
float dt_ = 0.1;
float Ka_ = 0.8 ;
float u = 0;
float circ_ = WHEEL_DIM * M_PI;
std_msgs::Float32 vel_msg_;

float setSPEED(float tar_vel, float cur_vel) { // rpm
  float err , output, min_signal, u_k ;
  double u_c,u_c_target;
  
  vel_msg_.data = cur_vel;
  
  err = tar_vel - cur_vel;
  P_err = Kp_ * err;
  I_err += Ki_ * err * dt_;
  D_err = (Kd_ * ((err - prev_err_) / dt_ ));
  
  //A_err += Ka_ * (prev_u_k - prev_u)  * dt_;
  
  u = P_err + cur_vel  + I_err + D_err ;//+ I_err  + D_err; //+ A_err ; + tar_vel 

  //sat(u(k))  saturation start
  if(u >2) u_k = 2 ;
  else if(u<=0) u_k = 0;
  else u_k = u;
  
  
  // u_k = tx_throttle_;
  
  /*inverse function */
  output = 2182.4 - (0.4*sqrt(2119700 - 808370*x))
  //u_c_target = 145.5942 - sqrt(21197 - (8083.7*tar_vel));
  
  //u_c = (-0.03602 + sqrt(pow(0.03602,2)+(4*0.0001237*(-u_k))))/(-2*(0.0001237));
  /**/

 /*output command*/
 /**/ 
 
  prev_u_k = u_k;
  prev_u = u;
  prev_err_ = err;

  if(tar_vel <= 0 ) output = ZERO_PWM;
  
  throttle_.writeMicroseconds(output);
  
  return output;
}
/*
   ANGLE to PWM
*/
void setANGLE() {
  static float output;
  float angle = tx_steer_;
  //if(IMU.update() > 0)
    //angle = -IMU.rpy[2];
  /*else
    angle = tx_steer_;*/
  output = (-angle * 12.0) + (float)STEER_CENTER;
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
   Encoder B interrupt service routine
*/
void getENB() {
  static boolean PINA = digitalRead(EN_PINA);
  static boolean PINB = digitalRead(EN_PINB);
  if (PINA ^ PINB) {
    EN_pos_ -= 1; // white - black
  } else {
    EN_pos_ += 1;   // white + black
  }
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
  //target_RPM = (tx_throttle_ / circ_)* 60; // RPM
  target_vel = tx_throttle_; // m/s
  //target_vel = test_throttle_;
  target_ANGLE = tx_steer_; // degree
  if(cumCountT_ == 0)
    cur_vel = 0;
  else  
     cur_vel = (float)EN_pos_ / TICK2CYCLE * ( SEC_TIME / ((float)cumCountT_*T_TIME)) * circ_; // m/s
  //cur_RPM = (float)EN_pos_ / TICK2CYCLE * ( 60.0 * SEC_TIME / ((float)cumCountT_*T_TIME)); // RPM
  output_vel = setSPEED(tx_throttle_, cur_vel);
  //output_angle = IMU.rpy[2];
  
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
  logfile_.print(P_err);
  logfile_.print(",");
  logfile_.print(I_err);
  logfile_.print(",");
  logfile_.print(D_err);
  logfile_.print(",");
  logfile_.println(u);
  /*
  logfile_.print(Kp_);
  logfile_.print(",");
  logfile_.println(Ki_);
  */
  /*
  logfile_.print(",");
  logfile_.print(tx_dist_);
  logfile_.print(",");
  logfile_.print(target_ANGLE);
  logfile_.print(",");
  logfile_.print(IMU.rpy[0]);
  logfile_.print(",");
  logfile_.print(IMU.rpy[1]);
  logfile_.print(",");
  logfile_.println(IMU.rpy[2]);*/
  logfile_.close();
    // CLEAR counter
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
ros::Subscriber<geometry_msgs::Twist> rosSubMsg("/twist_msg", &rosTwistCallback);
ros::Publisher rosPubMsg("/vel_msg", &vel_msg_);
/*
   Arduino setup()
*/
void setup() {
  nh_.initNode();
  nh_.subscribe(rosSubMsg);
  nh_.advertise(rosPubMsg);
  throttle_.attach(THROTTLE_PIN);
  steer_.attach(STEER_PIN);
  pinMode(EN_PINA, INPUT);
  pinMode(EN_PINB, INPUT);
  attachInterrupt(0, getENA, CHANGE);
  //attachInterrupt(1, getENB, CHANGE);
  IMU.begin();
  Serial.begin(BAUD_RATE);
  if(!SD.begin(10)){
    Serial.println("Card failed, or not present");
  } else {
    Serial.println("card initialized.");
    for(uint8_t i=0; i<100; i++){
      filename_[3] = i/10 + '0';
      filename_[4] = i%10 + '0';
      if(! SD.exists(filename_)){
        logfile_ = SD.open(filename_, FILE_WRITE);
        break;
      }
    }
    Serial.print("Logging to: ");
    Serial.print(filename_);
    //logfile_.println(filename_);
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
}
/*
   Arduino loop()
*/
void loop() {
  static unsigned long prevTime = 0;
  static unsigned long currentTime;
  float speed_vel ;
  static boolean flag_ = true;
  int count = 0;
  int stopcount = 0;
  static boolean stopflag_ = false;
  
  if(Serial.available() > 0) { // tx_throttle_
    // tx_steer_ = Serial.parseFloat();
    //speed_vel = Serial.parseFloat();
    tx_throttle_ = Serial.parseFloat();
    //flag_ = true;
    //Serial.print("Set velocity (m/s) : ");
    //Serial.println(tx_throttle_);
  }
  
  /*
  if(flag_) {
    delay(1000);
    tx_throttle_ = (float)speed_vel;
    delay(20.0/speed_vel*1000);
    tx_throttle_ = 0;
    delay(100000);
  }*/
  nh_.spinOnce();
  delay(1);
  /*
  currentTime = millis();
  if ((currentTime - prevTime) >= (CYCLE_TIME / 1000)) {
    rosPubMsg.publish(&vel_msg_);
    prevTime = currentTime;
    if (tx_throttle_ > 0  )
    {
        if ( flag_ ) {
            test_throttle_ = 0.5;
            flag_ = false;
        }
        if (count == 100 ){
            test_throttle_ = 0;
            count = 0;
            Ki_ += 0.01;
            stopflag_ = true;
        }
        if(flag_ == false && stopflag_ == false)count++;
    } else {
        test_throttle_ = 0;
    }
    if(stopflag_ == true )
    {
      stopcount++;
      if(stopcount == 100){
         flag_ = true;
         stopcount = 0;
         stopflag_ = false;
      }
    }
  }*/
}
