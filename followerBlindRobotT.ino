#include <IRremote.hpp>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>


LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

#define echoPin 13 // attach pin D2 Arduino to pin Echo of HC-SR04
#define pingPin 12 //attach pin D3 Arduino to pin Trig of HC-SR04
Servo s;

const int RSPD1 = 110;        //Right Wheel PWM.  Change this value so your car will go roughly straight
const int LSPD1 = 110;        //Left Wheel PWM


const int LWhFwdPin = 4;   // Connet to L298H
const int LWhBwdPin = A2;

const int RWhFwdPin = 7;
const int RWhBwdPin = A3;

const int LWhPWMPin = 5;
const int RWhPWMPin = 6;


void Extras_Setup();
void motorSetup();
void IR_Controller();
void PID_Controller();
void Echo();
void driveBck();
void driveFwd();
void turnLeft();


int gain = 19;
int delCntr = 0;
volatile long cntrL, cntrR;
volatile long LIntTime, RIntTime;


// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

void setup()
{
  Serial.begin(9600);
  motorSetup();
  Extras_Setup();

  cntrR = 0;
  cntrL = 0;
  LIntTime = 0;
  RIntTime = 0;
}


void loop() {
  PID_Controller();
  IR_Controller();
}
void forwardCertainDistance(int distance){
    distance = distance -10;
    cntrR = 0;
    cntrL = 0;
    LIntTime = 0;
    RIntTime = 0;
    driveFwd();
    while(cntrL<distance*2){
        PID_Controller();
        }
    off(); 
}
void parallelPark(int distance){
    delay(100);
    turnLeft();
    off();
    delay(100);
    Serial.println("In PP");
    off();
    cntrR = 0;
    cntrL = 0;
    LIntTime = 0;
    RIntTime = 0;
    int distanceSteps = 2*distance;
    //20.5cm one roation
    //40 steps one turn 
    driveFwd();  //forward until distanceSteps have been reached
    while(distanceSteps > cntrL){
          PID_Controller();  
        }
    off();
   delay(100);
   turnRight();
   off();
}

void IR_Controller() {
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    IrReceiver.printIRResultShort(&Serial);
    int code = IrReceiver.decodedIRData.command;
    Serial.println(code);
    if (code == 12 || code ==34) {
      driveFwd();
    }
    if (code == 8 || code ==33) {
      Stop();
    }
    if (code == 66 || code ==32) {
      driveBck();
    }
    if(code ==7){
      turnLeft();
    }
    if(code ==21){
      turnRight();
    }  
    int parkSignal =13;
    if (code == parkSignal){
          off();
          delay(300);
          IrReceiver.resume();
          while(1){
             if (IrReceiver.decode()) {
              IrReceiver.printIRResultShort(&Serial);
              int forwardDistance = IrReceiver.decodedIRData.command;;
              forwardCertainDistance(forwardDistance);       
              delay(100);
              break;
            }
      }
          IrReceiver.resume();
          delay(100);

          while(1){
            if (IrReceiver.decode()) {
              IrReceiver.printIRResultShort(&Serial);
              int parkingDistance = IrReceiver.decodedIRData.command;
              Serial.print("pa: ");
              Serial.println(parkingDistance);
              parallelPark(parkingDistance);
              break;
          }
        }
    }
    IrReceiver.resume();
  }
}

void PID_Controller() {
  long tmpLcntr, tmpRcntr;
  tmpLcntr = cntrL;
  tmpRcntr = cntrR;
  delCntr = abs(tmpLcntr - tmpRcntr);


  if (tmpLcntr > tmpRcntr)
  {
    analogWrite(RWhPWMPin, RSPD1);
    analogWrite(LWhPWMPin, max(LSPD1 - int(gain * delCntr + .5), 0));
  }
  else if (tmpLcntr < tmpRcntr)
  {
    analogWrite(LWhPWMPin, LSPD1);
    analogWrite(RWhPWMPin, max(RSPD1 - int(gain * delCntr + .5), 0));
  }
  else
  {
    analogWrite(RWhPWMPin, RSPD1);
    analogWrite(LWhPWMPin, LSPD1);
  }
}

void motorSetup() {
  pinMode(LWhFwdPin, OUTPUT);
  pinMode(LWhBwdPin, OUTPUT);
  pinMode(LWhPWMPin, OUTPUT);
  pinMode(RWhFwdPin, OUTPUT);
  pinMode(RWhBwdPin, OUTPUT);
  pinMode(RWhPWMPin, OUTPUT);

  digitalWrite(LWhFwdPin, LOW);
  digitalWrite(LWhBwdPin, LOW);
  digitalWrite(LWhPWMPin, LOW);

  digitalWrite(RWhFwdPin, LOW);
  digitalWrite(RWhBwdPin, LOW);
  digitalWrite(RWhPWMPin, LOW);

  attachInterrupt(digitalPinToInterrupt(2), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rightWhlCnt, CHANGE);

}

void Extras_Setup() {
  lcd.init();
  lcd.backlight();
  IrReceiver.begin(10, ENABLE_LED_FEEDBACK);
  pinMode(pingPin, OUTPUT); // Sets the pingPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  s.attach(11);
  s.write(180);
  s.write(90);
}

void leftWhlCnt()
{
  long intTime = micros();
  if (intTime > LIntTime + 1000L)
  {
    LIntTime = intTime;
    cntrL++;
  }

}

void rightWhlCnt()  
{
  long intTime = micros();
  if (intTime > RIntTime + 1000L)
  {
    RIntTime = intTime;
    cntrR++;
  }
}

void driveFwd() {
  off();
  delay(500);
  digitalWrite(LWhFwdPin, HIGH);   //run left wheel forward
  analogWrite(LWhPWMPin, LSPD1);
  digitalWrite(RWhFwdPin, HIGH);  //run right wheel forward
  analogWrite(RWhPWMPin, RSPD1);
}

void driveBck() {
  analogWrite(LWhPWMPin, 0);      // stop left wheel
  analogWrite(RWhPWMPin, 0);      // stop right wheel
  digitalWrite(LWhFwdPin, LOW);
  digitalWrite(RWhFwdPin, LOW);
  delay(500);
  digitalWrite(LWhBwdPin, HIGH);   //run left wheel back
  analogWrite(LWhPWMPin, LSPD1);
  digitalWrite(RWhBwdPin, HIGH);  //run right wheel back
  analogWrite(RWhPWMPin, RSPD1);


}

void Stop() {
  off();
}

void turnLeft() {
  off();
  digitalWrite(LWhBwdPin, HIGH);   
  digitalWrite(RWhFwdPin, HIGH);  

  long oldcntrL = cntrL;
  long oldcntrR = cntrR;
  LIntTime = 0;
  RIntTime = 0;
  cntrL = 0;
  cntrR = 0;
  analogWrite(LWhPWMPin, 140);
  analogWrite(RWhPWMPin, 130);
 
 
  while (cntrL+cntrR < 30) //carpet 40
  {
    
  }
  digitalWrite(LWhBwdPin, LOW);
  off();
  cntrL = oldcntrL;
  cntrR = oldcntrR;
}

void turnRight()
{
    Serial.print("Turning right");
    off();
    digitalWrite(RWhBwdPin, HIGH);   
    digitalWrite(LWhFwdPin, HIGH);   

    long oldcntrL = cntrL;
    long oldcntrR = cntrR;
    cntrL = 0;
    cntrR = 0;
    analogWrite(LWhPWMPin, 130);
    analogWrite(RWhPWMPin, 130);
  while (cntrL < (20)) //carpet 40
  {

  }
  digitalWrite(RWhBwdPin, LOW);   
  off();
  cntrL = oldcntrL;
  cntrR = oldcntrR;
}

void off() {
  digitalWrite(LWhFwdPin, LOW);
  digitalWrite(RWhFwdPin, LOW);
  analogWrite(LWhPWMPin, 0);      // stop FWD
  analogWrite(RWhPWMPin, 0);

  digitalWrite(LWhBwdPin, LOW);
  digitalWrite(RWhBwdPin, LOW);
  analogWrite(LWhPWMPin, 0);      // stop BWD
  analogWrite(RWhPWMPin, 0);
}
