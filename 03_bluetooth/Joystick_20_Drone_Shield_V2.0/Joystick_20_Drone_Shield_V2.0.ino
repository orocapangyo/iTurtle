//-----------------------------------------------
//-----------------------------------------------
#include <SoftwareSerial.h>

#define OPERATION_STEP_0  0
#define OPERATION_STEP_1  1
#define OPERATION_STEP_2  2
#define OPERATION_STEP_3  3
#define OPERATION_STEP_4  4
#define OPERATION_STEP_5  5
#define OPERATION_STEP_6  6
#define OPERATION_STEP_7  7
#define OPERATION_STEP_8  8
#define OPERATION_STEP_9  9
#define OPERATION_STEP_10  10

#define OPERATION_STEP_ERROR  100

String uartString = "";
unsigned int currentStep;
unsigned int oldStep;

SoftwareSerial bleSerial(A0, A1); // RX, TX
//-----------------------------------------------
//for Drone
unsigned char startBit = 0xf0;
unsigned char commandBit = 0xa1;
unsigned char roll = 100;
unsigned char pitch = 100;
unsigned char yaw = 100;
unsigned char throttle = 0;
unsigned char operationBit = 0x05;
unsigned char checkSum = 0;
//
unsigned int firstRoll;
unsigned int firstPitch;
//-----------------------------------------------
void initUart()
{
  uartString = "";
}

void initFlag()
{
  currentStep = OPERATION_STEP_1;
  oldStep = OPERATION_STEP_1;
}

void checkNextStep()
{
  oldStep = currentStep;
  currentStep = OPERATION_STEP_0;
}

void returnOldStep()
{
  currentStep = oldStep;
  currentStep++;
}

void checkCrLfProcess()
{
  while(bleSerial.available())
  {
    char inChar = bleSerial.read();
    uartString += inChar;
    //
    if( uartString.length() > 4
     && uartString.startsWith("\r\n")
     && uartString.endsWith("\r\n") )
    {
      returnOldStep();
      break;
    }
  }
}
//-----------------------------------------------
//for Drone
void checkThrottle()
{
  //throttle: 감소시 하강, 증가시 상승
  if(!digitalRead(6))
  {
    if(throttle > 59)
      throttle -= 20;
    else if(throttle > 3)
      throttle -= 4;
  }
  else if(!digitalRead(5))
  {
    if(throttle < 20)
      throttle = 20;
    else if(throttle < 181)
      throttle += 20;
  }
}

void checkYaw()
{
  //Yaw: 감소시 좌회전, 증가시 우회전
  if(!digitalRead(7))
  {
    yaw = 80;
  }
  else if(!digitalRead(8))
  {
    yaw = 120;
  }
  else
  {
     yaw = 100;
  }
}

void checkEmergency()
{
  //비상버튼 눌리면, 모터 회전 즉시 0으로 설정.
  if(!digitalRead(9))
    throttle = 0;
}

void checkRoll()
{
  //roll: 증가시 오른쪽 이동, 감소시 왼쪽 이동
  unsigned int secondRoll = analogRead(4);

  if(secondRoll < firstRoll - 450)
    roll = 75;
  else if(secondRoll < firstRoll - 350)
    roll = 80;
  else if(secondRoll < firstRoll - 250)
    roll = 85;
  else if(secondRoll < firstRoll - 150)
    roll = 90;
  else if(secondRoll < firstRoll - 50)
    roll = 95;
  else if(secondRoll < firstRoll + 50)
    roll = 100;
  else if(secondRoll < firstRoll + 150)
    roll = 105;
  else if(secondRoll < firstRoll + 250)
    roll = 110;
  else if(secondRoll < firstRoll + 350)
    roll = 115;
  else if(secondRoll < firstRoll + 450)
    roll = 120;
  else
    roll = 125;
}

void checkPitch()
{
  //pitch: 증가시 전진, 감소시 후진
  unsigned int secondPitch = analogRead(5);

  if(secondPitch < firstPitch - 450)
    pitch = 75;
  else if(secondPitch < firstPitch - 350)
    pitch = 80;
  else if(secondPitch < firstPitch - 250)
    pitch = 85;
  else if(secondPitch < firstPitch - 150)
    pitch = 90;
  else if(secondPitch < firstPitch - 50)
    pitch = 95;
  else if(secondPitch < firstPitch + 50)
    pitch = 100;
  else if(secondPitch < firstPitch + 150)
    pitch = 105;
  else if(secondPitch < firstPitch + 250)
    pitch = 110;
  else if(secondPitch < firstPitch + 350)
    pitch = 115;
  else if(secondPitch < firstPitch + 450)
    pitch = 120;
  else
    pitch = 125;
}

void sendDroneCommand()
{
  if(throttle == 0)
  {
    roll = 100;
    pitch = 100;
    yaw = 100;
  }
  //
  bleSerial.print("at+writeh0006");
  bleSerial.print(String(startBit,HEX));
  bleSerial.print(String(commandBit,HEX));
  //
  if(roll < 0x10)
    bleSerial.print("0" + String(roll,HEX));
  else 
    bleSerial.print(String(roll,HEX));
  if(pitch < 0x10)
    bleSerial.print("0" + String(pitch,HEX));
  else 
    bleSerial.print(String(pitch,HEX));
  if(yaw < 0x10)
    bleSerial.print("0" + String(yaw,HEX));
  else 
    bleSerial.print(String(yaw,HEX));
  if(throttle < 0x10)
    bleSerial.print("0" + String(throttle,HEX));
  else 
    bleSerial.print(String(throttle,HEX));
  bleSerial.print("0" + String(operationBit,HEX));
  //
  checkSum = commandBit + roll + pitch + yaw + throttle + operationBit;
  checkSum = checkSum & 0x00ff;
  if(checkSum < 0x10)
    bleSerial.print("0" + String(checkSum,HEX));
  else 
    bleSerial.print(String(checkSum,HEX));
  //
  bleSerial.print("\r");
}
//-----------------------------------------------
void setup()
{
  Serial.begin(9600);
  Serial.println("Test Started!");
  bleSerial.begin(9600);
  initFlag();
  initUart();

  for(int i = 5; i < 11; i++)
  {
    pinMode(i,INPUT);
    digitalWrite(i,HIGH);
  }
}

void loop()
{
  switch(currentStep)
  {
    case OPERATION_STEP_0:
      checkCrLfProcess();
    break;
    case OPERATION_STEP_1:
      delay(2000);
      bleSerial.flush();
      initFlag();
      initUart();
      while(bleSerial.available())
      {
        bleSerial.read();
      }
      currentStep++;
    break;
    case OPERATION_STEP_2:
      //wait, until press start button
      if(!digitalRead(9))
      {
        firstRoll = analogRead(4);
        firstPitch = analogRead(5);
        currentStep++;
      }
    break;
    case OPERATION_STEP_3:
      bleSerial.print("atd");
      //bleSerial.print("083a5c1f015b");
      bleSerial.print("b827ebb15add");
      bleSerial.print("\r");
      checkNextStep();
    break;
    case OPERATION_STEP_4:
      if(uartString.equals("\r\nOK\r\n"))
      {
        Serial.println("Wait Connect");
        delay(300);
        initUart();
        checkNextStep();
      }
      else
      {
        Serial.println("CONNECT 1 ERROR");
        initUart();
        currentStep = OPERATION_STEP_ERROR;
      }
    break;
    case OPERATION_STEP_5:
      if(uartString.startsWith("\r\nCONNECT "))
      {
        Serial.println("CONNECT OK");
        delay(300);
        initUart();
        currentStep++;
      }
      else
      {
        Serial.println("CONNECT 2 ERROR");
        initUart();
        currentStep = OPERATION_STEP_ERROR;
      }
    break;
    case OPERATION_STEP_6:
    {
      checkThrottle();
      checkRoll();
      checkPitch();
      checkYaw();
      checkEmergency();
      //
      sendDroneCommand();
      delay(10);
      //
      //Request Disconnect
      if(!digitalRead(10))
      {
        Serial.println("REQUEST DISCONNECT");
        delay(300);
        initUart();
        currentStep++;
      }
    }
    break;
    case OPERATION_STEP_7:
      delay(1000);
      bleSerial.flush();
      initUart();
      while(bleSerial.available())
      {
        bleSerial.read();
      }
      initUart();
      bleSerial.print("ath\r");
      checkNextStep();
    break;
    case OPERATION_STEP_8:
      if(uartString.equals("\r\nOK\r\n"))
      {
        Serial.println("Wait Disconnect");
        delay(300);
        initUart();
        checkNextStep();
      }
      else
      {
        Serial.println("DISCONNECT 1 ERROR");
        initUart();
        currentStep = OPERATION_STEP_ERROR;
      }
    break;
    case OPERATION_STEP_9:
      if(uartString.startsWith("\r\nDISCONNECT"))
      {
        Serial.println("DISCONNECT 1 OK");
        delay(300);
        initUart();
        checkNextStep();
      }
      else
      {
        Serial.println("DISCONNECT 2 ERROR");
        initUart();
        currentStep = OPERATION_STEP_ERROR;
      }
    break;
    case OPERATION_STEP_10:
      if(uartString.startsWith("\r\nREADY"))
      {
        Serial.println("DISCONNECT 2 OK");
        delay(300);
        initUart();
        currentStep = OPERATION_STEP_2;
      }
      else
      {
        Serial.println("DISCONNECT 3 ERROR");
        initUart();
        currentStep = OPERATION_STEP_ERROR;
      }
    break;
    default:
      if(bleSerial.available())
        Serial.write(bleSerial.read());
      if(Serial.available())
        bleSerial.write(Serial.read());
    break;
  }
}

