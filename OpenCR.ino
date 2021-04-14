#include <stdio.h>
#include <Servo.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#define BAUD_RATE     (57600)
#define TICK2CYCLE    (65) // 65 ticks(EN_pos_) = 1 wheel cycle
#define CYCLE_TIME    (200000) // us
#define STEER_TIME    (333000) // us
#define SPEED_TIME    (100000) // us
#define SEC_TIME      (1000000) // us
#define SHIFT_PIN     (11)
#define STEER_PIN     (10)
#define THROTTLE_PIN  (9)
#define EN_PINA       (3)
#define EN_PINB       (2)
#define WHEEL_DIM     (0.085) // m

#define SHIFT_VAL     (1100)
#define MAX_SPEED     (4)  // m/s


#define TRUCK_0

#ifdef TRUCK_0
#define MAX_PWM       (1100)
#define MIN_PWM       (1450)
#define ZERO_PWM      (1480)
#define MAX_STEER     (1800)
#define MIN_STEER     (1200)
#define STEER_CENTER  (1500)
#endif

#ifdef TRUCK_1
#define MAX_PWM       (2000)
#define MIN_PWM       (1595)
#define MAX_STEER     (1800)
#define MIN_STEER     (1200)
#define STEER_CENTER  (1500)
#endif

#ifdef TRUCK_2
#define MAX_PWM       (1100)
#define MIN_PWM       (1475)
#define MAX_STEER     (1800)
#define MIN_STEER     (1200)
#define STEER_CENTER  (1500)
#endif

#ifdef TRUCK_4
#define MAX_PWM       (1100)
#define MIN_PWM       (1450)
#define ZERO_PWM      (1480)
#define MAX_STEER     (1800)
#define MIN_STEER     (1200)
#define STEER_CENTER  (1300)
#endif

/*
   PWM variable
*/

Servo shift_;
Servo throttle_;
Servo steer_;
float tx_throttle_;
float tx_steer_;
int EN_pos_;

HardwareTimer Timer1(TIMER_CH1); // Check EN
HardwareTimer Timer2(TIMER_CH2); // SPEED TIMER

/*
   ros Subscribe Callback Function
*/

void rosTwistCallback(const geometry_msgs::Twist& msg) {
  tx_throttle_ = msg.linear.x;
  tx_steer_ = msg.angular.z;
}

/*
   SPEED to RPM
*/

float cur_RPM_, cur_ROUND_;
float prev_err_;
float result_;

float Kp_ = 5.0;
float Ki_ = 0.0001;
float Kd_ = 0.025;
float dt_ = 0.25;
float circ_ = WHEEL_DIM * M_PI;

std_msgs::Float32 vel_msg_;

void setSPEED() { // m/s
  float tar_vel = tx_throttle_;
  if (tar_vel > MAX_SPEED)
    tar_vel = MAX_SPEED;
  float err, I_err, D_err, output;
  // CYCLE_TIME
  float tar_vel_CT = (tar_vel / circ_) / (SEC_TIME / CYCLE_TIME);
  float cur_vel_CT = cur_ROUND_ * circ_;
  vel_msg_.data = cur_vel_CT;

  err = tar_vel_CT - cur_vel_CT;
  I_err += err * dt_;
  D_err = (err - prev_err_) / dt_;
  prev_err_ = err;


  if (tar_vel <= 0) {
    result_ = output = 0;
    I_err = 0;
  } else {
    output = (Kp_ * err) + (Kd_ * D_err) + (Ki_ * I_err);
  }
  if (output == 0)
  {
    output = ZERO_PWM;
  } else {
    if (MIN_PWM < MAX_PWM) {
      result_ += output;
      if (result_ > 1800)
        result_ = 1800;
      output = result_ + MIN_PWM;
      if (output >= MAX_PWM)
        output = MAX_PWM;
    } else {
      result_ -= output;
      if (result_ < -1800)
        result_ = -1800;
      output = result_ + MIN_PWM;
      if (output <= MAX_PWM)
        output = MAX_PWM;
    }
  }
  throttle_.writeMicroseconds(output);

  Serial.print("target / current / PWM : ");
  Serial.print(tar_vel_CT*1000);
  Serial.print(" / ");
  Serial.print(cur_vel_CT*1000);
  Serial.print(" / ");
  Serial.println(output);
}

/*
   ANGLE to PWM
*/

void setANGLE(float angle) {
  float output;
  output = angle * 12 + STEER_CENTER;
  steer_.writeMicroseconds(output);

  /*
    Serial.print("Degree/PWM : ");
    Serial.print(tar_val);
    Serial.print(" / ");
    Serial.println(output);
  */
}

/*
   Encoder A interrupt service routine
*/

void getENA() {
  if (digitalRead(EN_PINA) == HIGH) {
    if (digitalRead(EN_PINB) == LOW) {
      EN_pos_ += 1;
    }
    else {
      EN_pos_ -= 1;
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
}

/*
   Encoder B interrupt service routine
*/

void getENB() {
  if (digitalRead(EN_PINB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(EN_PINA) == HIGH) {
      EN_pos_ += 1;
    }
    else {
      EN_pos_ -= 1;
    }
  }
  else {
    if (digitalRead(EN_PINA) == LOW) {
      EN_pos_ += 1;
    }
    else {
      EN_pos_ -= 1;
    }
  }
}

/*
   RPM Check Function
*/

void CheckEN() {
  cur_RPM_ = (float)EN_pos_ / TICK2CYCLE * (60 * SEC_TIME / CYCLE_TIME);
  cur_ROUND_ = (float)EN_pos_ / TICK2CYCLE * (SEC_TIME / CYCLE_TIME);
  //Serial.println(EN_pos_);
  EN_pos_ = 0;
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

  shift_.attach(SHIFT_PIN);
  throttle_.attach(THROTTLE_PIN);
  steer_.attach(STEER_PIN);

  pinMode(EN_PINA, INPUT);
  pinMode(EN_PINB, INPUT);
  attachInterrupt(0, getENA, CHANGE);
  attachInterrupt(1, getENB, CHANGE);

  Timer1.stop();
  Timer1.setPeriod(CYCLE_TIME);
  Timer1.attachInterrupt(CheckEN);
  Timer1.start();

  Timer2.stop();
  Timer2.setPeriod(SPEED_TIME);
  Timer2.attachInterrupt(setSPEED);
  Timer2.start();

  shift_.writeMicroseconds(SHIFT_VAL);
  steer_.writeMicroseconds(0);
  Serial.begin(BAUD_RATE);
  Serial.print("[OpenCR] setup()");
}

/*
   Arduino loop()
*/

void loop() {
  static unsigned long prevTime = 0;
  unsigned long currentTime;

  /*
  if(Serial.available() > 0) { // tx_throttle_
    tx_throttle_ = Serial.parseFloat();
    //Serial.print("Set velocity (m/s) : ");
    //Serial.println(tx_throttle_);
  }
  */
  /*
  if(Serial.available() > 0) { // tx_throttle_
    tx_steer_ = Serial.parseFloat();
    Serial.print("Set velocity (m/s) : ");
    Serial.println(tx_steer_);
  }
  */
  
  setANGLE(tx_steer_);
  shift_.writeMicroseconds(SHIFT_VAL);

  nh_.spinOnce();
  delay(1);

  currentTime = millis();
  if ((currentTime - prevTime) >= (SPEED_TIME / 1000)) {
    rosPubMsg.publish(&vel_msg_);
    prevTime = currentTime;
  }
}
