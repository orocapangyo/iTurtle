#include <MPU9250_asukiaaa.h>

/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 * 
 * 
 * 기본 메모리
 * 스케치는 프로그램 저장 공간 12668 바이트(44%)를 사용. 최대 28672 바이트.
 * 전역 변수는 동적 메모리 1761바이트(68%)를 사용, 799바이트의 지역변수가 남음.  최대는 2560 바이트. 
 * 
 * Neo Pixel LED Control
 * 스케치는 프로그램 저장 공간 13598 바이트(47%)를 사용. 최대 28672 바이트.
 * 전역 변수는 동적 메모리 1794바이트(70%)를 사용, 766바이트의 지역변수가 남음.  최대는 2560 바이트. 
 * 
 * Motor Control
 * 스케치는 프로그램 저장 공간 13844 바이트(48%)를 사용. 최대 28672 바이트.
 * 전역 변수는 동적 메모리 1794바이트(70%)를 사용, 766바이트의 지역변수가 남음.  최대는 2560 바이트. 
 * 
 * MPU9250
 * 스케치는 프로그램 저장 공간 22106 바이트(77%)를 사용. 최대 28672 바이트.
 * 전역 변수는 동적 메모리 2301바이트(89%)를 사용, 259바이트의 지역변수가 남음.  최대는 2560 바이트.
 * 
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <Adafruit_NeoPixel.h>



// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        6 // On Trinket or Gemma, suggest changing this to 1
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels
#define NUMPIXELS 5

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


ros::NodeHandle  nh;


int STBY = 10; //standby

//Motor A
int PWMA = 3; //Speed control 
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction

//Motor B
int PWMB = 5; //Speed control
int BIN1 = 11; //Direction
int BIN2 = 12; //Direction

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)


  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  Serial.begin(115200);


  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();


  
}

void loop()
{  

  uint8_t sensorId;
  if (mySensor.readId(&sensorId) == 0) {
    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println("Cannot read sensorId");
  }

  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.println("accelX: " + String(aX));
    Serial.println("accelY: " + String(aY));
    Serial.println("accelZ: " + String(aZ));
    Serial.println("accelSqrt: " + String(aSqrt));
  } else {
    Serial.println("Cannod read accel values");
  }

  if (mySensor.gyroUpdate() == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.println("gyroX: " + String(gX));
    Serial.println("gyroY: " + String(gY));
    Serial.println("gyroZ: " + String(gZ));
  } else {
    Serial.println("Cannot read gyro values");
  }

  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.println("magX: " + String(mX));
    Serial.println("maxY: " + String(mY));
    Serial.println("magZ: " + String(mZ));
    Serial.println("horizontal direction: " + String(mDirection));
  } else {
    Serial.println("Cannot read mag values");
  }  

  
  pixels.clear(); // Set all pixel colors to 'off'pixels.clear(); // Set all pixel colors to 'off'
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(0, 150, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
  }

  move(1, 255, 1); //motor 1, full speed, left
  move(2, 255, 1); //motor 2, full speed, left

  delay(1000); //go for 1 second
  stop(); //stop
  
  nh.spinOnce();
  delay(1);
}

void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop(){
//enable standby  
  digitalWrite(STBY, LOW); 
}

