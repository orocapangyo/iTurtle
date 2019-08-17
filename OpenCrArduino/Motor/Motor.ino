/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(3); //attach it to pin 9

  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(11, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
}

void loop(){
  nh.spinOnce();
  delay(1);

  motorMode(100, 1);

  //servo.write(0);
  //delay(1000);
  //servo.write(180);
  //delay(1000);
}

void motorMode(int speedValue, int dir){

  digitalWrite(4,LOW);
  digitalWrite(7,HIGH);
  
  analogWrite(11,255);
}

