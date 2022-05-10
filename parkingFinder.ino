
/*Brendan McLaughlin
   LAB06 Section A
*/


//Interrupts have to be on pit 2 and 3
//2:24 FWD
//5:28 STOP
//8:82 BWD
#define DECODE_NEC
#include <IRremote.hpp>
#include <Servo.h>
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD
Servo myservo;  // servo object to control ultrasonic servo
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

const int pingPin = 12; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 13;
const int RSPD1 = 150;        //Right Wheel PWM
const int LSPD1 = 150;        //Left Wheel PWM
const int LWhFwdPin = 4;   // Connet to L298H
const int LWhBwdPin = A2;
const int LWhPWMPin = 5;
const int RWhFwdPin = 7;
const int RWhBwdPin = A3;
const int RWhPWMPin = 6;

int state = 0;
int x = 0 ;
int gain = 19;
bool forward = false;
bool backward = false;
int delCntr = 0;
volatile long cntrL, cntrR;
volatile long LIntTime, RIntTime;
long stopTime;
long measure = 0;
const int ultraServoPin = 9; // Connet to Servo Mortor
boolean parallelOff = true;

int pos = 0;    // variable to store the servo position
void setup()
{
  lcd.init();
  lcd.backlight();

  pinMode(10, INPUT);
  pinMode(LWhFwdPin, OUTPUT);
  pinMode(LWhBwdPin, OUTPUT);
  pinMode(LWhPWMPin, OUTPUT);
  pinMode(RWhFwdPin, OUTPUT);
  pinMode(RWhBwdPin, OUTPUT);
  pinMode(RWhPWMPin, OUTPUT);


  Serial.begin(9600);
  //
  digitalWrite(LWhFwdPin, LOW);
  digitalWrite(LWhBwdPin, LOW);
  digitalWrite(LWhPWMPin, LOW);

  digitalWrite(RWhFwdPin, LOW);
  digitalWrite(RWhBwdPin, LOW);
  digitalWrite(RWhPWMPin, LOW);
  myservo.attach(ultraServoPin);

  IrReceiver.begin(10, ENABLE_LED_FEEDBACK); // Start the receiver
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rightWhlCnt, CHANGE);

  cntrR = 0;
  cntrL = 0;
  LIntTime = 0;
  RIntTime = 0;
  stopTime = micros() + 10 * 1000000;

}


void loop()
{
  //lcd.setCursor(2, 0); // Set the cursor on the third column and first row.
  long inches;
  inches = ultraDistance();
  Serial.println("test");
  Serial.print("Inches: ");
  lcd.clear();
  Serial.println(inches);
  lcd.print(inches );
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    IrReceiver.printIRResultShort(&Serial);
    Serial.print("IR Reading (DEC): ");
    Serial.println(IrReceiver.decodedIRData.command);
    x = IrReceiver.decodedIRData.command;
    state = x;
    Serial.print("Test Value: ");
    Serial.println(x);
    IrReceiver.resume();

    if (state == 28) {
      digitalWrite(LWhFwdPin, LOW);
      digitalWrite(RWhFwdPin, LOW);
      analogWrite(LWhPWMPin, 0);      // stop FWD
      analogWrite(RWhPWMPin, 0);

      digitalWrite(LWhBwdPin, LOW);
      digitalWrite(RWhBwdPin, LOW);
      analogWrite(LWhPWMPin, 0);      // stop BWD
      analogWrite(RWhPWMPin, 0);
      forward = false;
      backward = false;
    }

    if (state == 24) {
      digitalWrite(LWhBwdPin, LOW);
      digitalWrite(RWhBwdPin, LOW);
      analogWrite(LWhPWMPin, 0);      // stop BWD
      analogWrite(RWhPWMPin, 0);

      digitalWrite(LWhFwdPin, HIGH);   //run left wheel FWD
      analogWrite(LWhPWMPin, LSPD1);
      digitalWrite(RWhFwdPin, HIGH);  //run right wheel FWD
      analogWrite(RWhPWMPin, RSPD1);
      forward = true;
      backward = false;
      parallelOff = true;

    }

    if (state == 82) {
      digitalWrite(LWhFwdPin, LOW);   //
      digitalWrite(RWhFwdPin, LOW);
      analogWrite(LWhPWMPin, 0);      // stop FWD
      analogWrite(RWhPWMPin, 0);

      digitalWrite(LWhBwdPin, HIGH);   //run left wheel BWD
      analogWrite(LWhPWMPin, LSPD1);
      digitalWrite(RWhBwdPin, HIGH);  //run right wheel BWD
      analogWrite(RWhPWMPin, RSPD1);
      forward = false;
      backward = true;
      parallelOff = true;
    }

    if (state == 21) { //parrallel park.
      turnright();
      off();
      delay(1000);
      forwardToWall();
      off();
      delay(300);
      turnleft();
      off();
      parallelOff = false;
      //go straight until close to wall
    }
  }

  //forward = true;
  if (inches < 8 && parallelOff && forward) {
    boolean rightsafe = false;
    boolean leftsafe = false;
    off();
    delay(500);
    myservo.write(180); //check left
    delay(1000);
    if (ultraDistance()  < 8) {
      off();
      forward = true;
      rightsafe = true;
    }
    delay(1000);
    myservo.write(0);
    delay(1000);

    if (ultraDistance() < 8) {
      off();
      forward = true;
      leftsafe = true;
    }
    if (leftsafe) {
      turnleft();
      delay(1000);
      myservo.write(90);
      digitalWrite(LWhFwdPin, HIGH);   //run left wheel FWD
      analogWrite(LWhPWMPin, LSPD1);
      digitalWrite(RWhFwdPin, HIGH);  //run right wheel FWD
      analogWrite(RWhPWMPin, RSPD1);
      forward = true;
      //delay(100);
    }
    else {
      turnright();
      delay(500);
      delay(1000);
      myservo.write(90);
      digitalWrite(LWhFwdPin, HIGH);   //run left wheel FWD
      analogWrite(LWhPWMPin, LSPD1);
      digitalWrite(RWhFwdPin, HIGH);  //run right wheel FWD
      analogWrite(RWhPWMPin, RSPD1);
      forward = true;
      //delay();

    }
  }


  if (forward) {
    Serial.println("FORWARD");
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

  if (backward) {

    long tmpLcntr, tmpRcntr;
    tmpLcntr = cntrL;
    tmpRcntr = cntrR;
    delCntr = abs(tmpLcntr - tmpRcntr);
    if (tmpLcntr > tmpRcntr)
    {
      analogWrite(RWhPWMPin, RSPD1);
      analogWrite(LWhPWMPin, max(LSPD1 + int(gain * delCntr + .5), 0)); //Backwards so use plus
    }
    else if (tmpLcntr < tmpRcntr)
    {
      Serial.println("24");
      analogWrite(LWhPWMPin, LSPD1);
      analogWrite(RWhPWMPin, max(RSPD1 + int(gain * delCntr + .5), 0)); //Backwards so use plus
    }
    else
    {
      analogWrite(RWhPWMPin, RSPD1);
      analogWrite(LWhPWMPin, LSPD1);
    }
  }
  cntrR = 0;
  cntrL = 0;
  LIntTime = 0;
  RIntTime = 0;

}

void turnright()
{
  Serial.print("Turning right");
  off();
  digitalWrite(RWhBwdPin, HIGH);   //
  long oldcntrL = cntrL;
  long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  //while (cntrL<(cntrR+25))
  while (cntrR < (cntrL + 30)) //carpet 40
  {
    analogWrite(LWhPWMPin, 0);
    analogWrite(RWhPWMPin, RSPD1);
  }
  digitalWrite(RWhBwdPin, LOW);   //
  cntrL = oldcntrL;
  cntrR = oldcntrR;
  //      cntrR = 0;
  //      cntrL = 0;
  //      LIntTime = 0;
  //      RIntTime = 0;
}

void turnleft()
{
  Serial.print("Turning left");

  off();
  digitalWrite(LWhBwdPin, HIGH);   //

  long oldcntrL = cntrL;
  long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrL < (cntrR + 30)) //carpet 40
  {
    analogWrite(RWhPWMPin, 0);
    analogWrite(LWhPWMPin, LSPD1);
    analogWrite(RWhPWMPin, 0);
    analogWrite(LWhPWMPin, RSPD1);
  }
  digitalWrite(LWhBwdPin, LOW);   //
  cntrL = oldcntrL;
  cntrR = oldcntrR;
  //      cntrR = 0;
  //      cntrL = 0;
  //      LIntTime = 0;
  //      RIntTime = 0;
}

void forwardToWall() {
  digitalWrite(LWhFwdPin, HIGH);   //run left wheel FWD
  analogWrite(LWhPWMPin, LSPD1 - 30);
  digitalWrite(RWhFwdPin, HIGH);  //run right wheel FWD
  analogWrite(RWhPWMPin, RSPD1 - 30);
  cntrR = 0;
  cntrL = 0;
  LIntTime = 0;
  RIntTime = 0;
  while (true) {
    long tempinch;
    tempinch = ultraDistance();
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
    if (tempinch < 5) {
      off();
      break;
    }
  }

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

void rightWhlCnt()  // Complete this ISR
{
  long intTime = micros();
  if (intTime > RIntTime + 1000L)
  {
    RIntTime = intTime;
    cntrR++;
  }

}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long ultraDistance() {
  delay(100);
  long duration, inches, cm;
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  inches = microsecondsToInches(duration);
  return inches;

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
