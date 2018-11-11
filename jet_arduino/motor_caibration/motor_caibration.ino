#include "DualVNH5019MotorShield.h"
#include "NewPing.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include "Wire.h"

/*
 * CONSTANTS
 */
//Motor Constants
#define MAX_SPEED 400
#define BRAKE_POWER 400
#define MOTOR_TIMEOUT_MS 1000

//Encoder Constants
#define ENCODER_INTERVAL 20

#define encoder_pin_left_interrupt 5
#define encoder_pin_right_interrupt 4

#define encoder_pin_left_A 18
#define encoder_pin_right_A 19
#define encoder_pin_left_B 20
#define encoder_pin_right_B 21

//Ros NodeHandler
ros::NodeHandle  nh;

//Encoders
signed long encoderTimer;
std_msgs::Int64 encoder_left_value, encoder_right_value;
ros::Publisher encoder_left_pub("arduino/encoder_left_value", &encoder_left_value);
ros::Publisher encoder_right_pub("arduino/encoder_right_value", &encoder_right_value);

//Set speed publisher
std_msgs::Int64 speed_left, speed_right;
ros::Publisher speed_left_pub("/arduino/speed_left", &speed_left);
ros::Publisher speed_right_pub("/arduino/speed_right", &speed_right);

void motor_right_speed_cb(const std_msgs::Int16 &cmd_msg) {
    motorTimer = millis();
    md.setM2Speed(m2Speed);
    //md.setM2Speed(cmd_msg.data);
    if ((m2Speed == 0) && (cmd_msg.data == 0))
      md.setM2Brake(BRAKE_POWER);   
}

void motor_left_speed_cb(const std_msgs::Int16 &cmd_msg) {
    motorTimer = millis();
    md.setM1Speed(m1Speed);
    //md.setM1Speed(cmd_msg.data);
    if ((m1Speed == 0) && (cmd_msg.data == 0))
      md.setM1Brake(BRAKE_POWER);
}

void encoder_left_cb() {
  if(digitalRead(encoder_pin_left_B) == HIGH) {
    if(digitalRead(encoder_pin_left_A) == LOW) {
      encoder_left_value.data--;
    }
    else{
    encoder_left_value.data++;
    }
  }
  else{
    if(digitalRead(encoder_pin_right_A) == LOW){
      encoder_left_value.data++;
    }
    else{
      encoder_left_value.data--;
    }
  }  
}

void encoder_right_cb() {
  if(digitalRead(encoder_pin_right_A) == HIGH) {
    if(digitalRead(encoder_pin_right_B) == LOW) {
      encoder_right_value.data--;
    }
    else{
    encoder_right_value.data++;
    }
  }
  else{
    if(digitalRead(encoder_pin_right_B) == LOW){
      encoder_right_value.data++;
    }
    else{
      encoder_right_value.data--;
    }
  }  
}

void setup() {
  // put your setup code here, to run once:
  md.init();
  nh.getHardware()->setBaud(115200);

  nh.initNode();

  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);
  
  nh.advertise(speed_left_pub);
  nh.advertise(speed_right_pub);

  pinMode(encoder_pin_left_A, INPUT);
  digitalWrite(encoder_pin_left_A, LOW);
  
  pinMode(encoder_pin_right_A, INPUT);
  digitalWrite(encoder_pin_right_A, LOW);
  
  pinMode(encoder_pin_left_B, INPUT);
  digitalWrite(encoder_pin_left_B, LOW);
  
  pinMode(encoder_pin_right_B, INPUT);
  digitalWrite(encoder_pin_right_B, LOW);
  
  attachInterrupt(encoder_pin_left_interrupt, encoder_left_cb, RISING); //digital pin 18, //5
  attachInterrupt(encoder_pin_right_interrupt, encoder_right_cb, RISING); //digital pin 19, //4
  
  encoder_left_value.data = 0;
  encoder_right_value.data = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  for (int i=-400; i<=400; i++){
    speed_left_pub.publish(&speed_left);
    speed_right_pub.publish(&speed_right);
      if (millis() >= currentTimer) {
    motor_right_current.data = md.getM2CurrentMilliamps();
    motor_left_current.data = md.getM1CurrentMilliamps();

    motor_right_current_pub.publish(&motor_right_current);
    motor_left_current_pub.publish(&motor_left_current);
    currentTimer += CURRENT_INTERVAL;}

  if (millis() >= encoderTimer) {
    encoder_left_pub.publish(&encoder_left_value);
    encoder_right_pub.publish(&encoder_right_value);
    encoderTimer += ENCODER_INTERVAL;}

  if (millis() > motorTimer + MOTOR_TIMEOUT_MS) {
    md.setM1Brake(BRAKE_POWER);
    md.setM2Brake(BRAKE_POWER);}
    
  delay(10);
  nh.spinOnce();
}
}
