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
#include "geometry_msgs/Twist.h"

float x; 


ros::NodeHandle  nh;

Servo servo;


// RIGHT Motor
#define leftMotor_A    12
#define leftMotor_B    8
#define leftMotor_PWM  13

// LEFT Motor
#define rightMotor_A    7
#define rightMotor_B    4
#define rightMotor_PWM  11


#define MOTOR_FW        0
#define MOTOR_BW        1
#define MOTOR_RW        2
#define MOTOR_LW        3
#define MOTOR_STOP      4


int16_t rmPWM;
int16_t lmPWM;



void velCallback(  const geometry_msgs::Twist& vel)
{
    if(vel.linear.x > 0)
        setPWM( 100 ,  100);
    else if(vel.linear.z > 0)
       setPWM( -100 ,  -100);     
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);


void setMotor() 
{
  pinMode( rightMotor_A, OUTPUT );
  pinMode( rightMotor_B, OUTPUT );
 
  pinMode( leftMotor_A, OUTPUT );
  pinMode( leftMotor_B, OUTPUT );

  setDirout_rightMotor(MOTOR_STOP);
  setDirout_leftMotor(MOTOR_STOP);

  setPWM_rightMotor(0);
  setPWM_leftMotor(0);
}



void setPWM(int16_t rmPWM, int16_t lmPWM)
{
  if ( rmPWM >  255 )  rmPWM =  255;
  if ( rmPWM < -255 )  rmPWM = -255;

  if ( lmPWM >  255 ) lmPWM =  255;
  if ( lmPWM < -255 ) lmPWM = -255;

  if ( rmPWM == 0 )
  {
    setDirout_rightMotor ( MOTOR_STOP );
    setPWM_rightMotor ( 0 );
  }
  else if ( rmPWM > 0 )
  {
    setDirout_rightMotor ( MOTOR_FW );
    setPWM_rightMotor ( rmPWM);
  }
  else
  {
    setDirout_rightMotor ( MOTOR_BW );
    setPWM_rightMotor ( - rmPWM );
  }

  if ( lmPWM == 0 )
  {
    setDirout_leftMotor ( MOTOR_STOP );
    setPWM_leftMotor ( 0 );
  }
  else if (lmPWM > 0 )
  {
    setDirout_leftMotor ( MOTOR_FW );
    setPWM_leftMotor ( lmPWM);
  }
  else
  {
    setDirout_leftMotor ( MOTOR_BW );
    setPWM_leftMotor ( - lmPWM );
  }
}



void setPWM_rightMotor (uint16_t pwmData)
{
  if(pwmData > 255) pwmData = 255;

  rmPWM = pwmData;

  analogWrite( rightMotor_PWM, pwmData);  
}



void setPWM_leftMotor(uint16_t pwmData)
{
  if(pwmData > 255) pwmData = 255;

  lmPWM = pwmData;

  analogWrite( leftMotor_PWM, pwmData);  
}



void setDirout_leftMotor(uint8_t DirData) 
{
  switch ( DirData )
  {
    case MOTOR_FW:
      digitalWrite( leftMotor_A, LOW );
      digitalWrite( leftMotor_B, HIGH);
      break;

    case MOTOR_BW:
      digitalWrite( leftMotor_A, HIGH );
      digitalWrite( leftMotor_B, LOW );
      break;

    case MOTOR_STOP:
      digitalWrite( leftMotor_A, LOW );
      digitalWrite( leftMotor_B, LOW );
      break;

    default:
      digitalWrite( leftMotor_A, LOW );
      digitalWrite( leftMotor_B, LOW );
      break;
  }
}


void setDirout_rightMotor(uint8_t DirData) 
{
  switch ( DirData )
  {
    case MOTOR_FW:
      digitalWrite( rightMotor_A, LOW );
      digitalWrite( rightMotor_B, HIGH);
      break;

    case MOTOR_BW:
      digitalWrite( rightMotor_A, HIGH);
      digitalWrite( rightMotor_B, LOW );
      break;

    case MOTOR_STOP:
      digitalWrite( rightMotor_A, LOW );
      digitalWrite( rightMotor_B, LOW );
      break;

    default:
      digitalWrite( rightMotor_A, LOW );
      digitalWrite( rightMotor_B, LOW );
      break;
  }
}


void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(3); //attach it to pin 9

  setMotor();
}

void loop(){
  nh.spinOnce();
  delay(1);



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

