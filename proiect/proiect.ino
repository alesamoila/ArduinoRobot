#include <Servo.h>
#include <SoftwareSerial.h>
// Pinii motor 1
#define mpin00 5
#define mpin01 6
// Pinii motor 2
#define mpin10 3
#define mpin11 11
#define echo 12
#define trig 9
#define txPin 4
#define rxPin 7

SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

int stopped = 0;
String text = " ";
long timp1, timp2, timp3;
Servo srv;


void setup() {

  mySerial.begin(9600);
  Serial.begin(9600);
  // configurarea pinilor motor ca iesire, initial valoare 0
  digitalWrite(mpin00, 0);
  digitalWrite(mpin01, 0);
  digitalWrite(mpin10, 0);
  digitalWrite(mpin11, 0);
  pinMode (mpin00, OUTPUT);
  pinMode (mpin01, OUTPUT);
  pinMode (mpin10, OUTPUT);
  pinMode (mpin11, OUTPUT);
  pinMode (echo, INPUT);
  pinMode (trig, OUTPUT);

}
// Funcție pentru controlul unui motor
// Intrare: pinii m1 și m2, direcția și viteza
void StartMotor (int m1, int m2, int forward, int speed)
{

  if (speed == 0) // oprire
  {
    digitalWrite(m1, 0);
    digitalWrite(m2, 0);
  }
  else
  {
    if (forward)
    {
      digitalWrite(m2, 0);
      analogWrite(m1, speed); // folosire PWM
    }
    else
    {
      digitalWrite(m1, 0);
      analogWrite(m2, speed);
    }
  }
}
// Funcție de siguranță
// Execută oprire motoare, urmată de delay
void delayStopped(int ms)
{
  StartMotor (mpin00, mpin01, 0, 0);
  StartMotor (mpin10, mpin11, 0, 0);
  delay(ms);
}

void setServo(int pin, int angle) {
  srv.attach(pin);
  srv.write(angle);
  delay(1000);
  srv.detach();
}

void forward(int q) {

  stopped = 1;
  StartMotor (mpin00, mpin01, 1, 128);
  StartMotor (mpin10, mpin11, 1, 128);

  delay (q); // Cât timp e motorul pornit
}

void right(int q) {
  StartMotor (mpin00, mpin01, 1, 64);
  StartMotor (mpin10, mpin11, 0, 64);

  delay(q);
}

void left(int q) {
  StartMotor (mpin00, mpin01, 0, 64);
  StartMotor (mpin10, mpin11, 1, 64);

  delay(q);
}

double calcDistance() {

  digitalWrite(trig, 1);
  delayMicroseconds(10);
  digitalWrite(trig, 0);
  double distance;
  timp2 = pulseIn(echo, HIGH, 100000);
  digitalWrite(trig, 0);
  distance = (double)timp2 * 0.0343 / 2;
  return distance;
}

void back(int q) {

  stopped = -1;
  StartMotor (mpin00, mpin01, 0, 128);
  StartMotor (mpin10, mpin11, 0, 128);
  delay(q);
}

void loop() {

  double distance = calcDistance();
  mySerial.println(distance);

  if (mySerial.available()) {
    text = (String) mySerial.readString();

    if (text.substring(0, 7) == "forward") {
      setServo(8, 90);
    }
    if (text.substring(0, 6) == "left") {
      setServo(8, 180);
    }
    if (text.substring(0, 7) == "right") {
      setServo(8, 0);
    }

    if (text.substring(0, 12) == "engineForward") {
      if (stopped != 0) {
        delayStopped(500);
      }
      forward(500);
    }
    if (text.substring(0, 11) == "engineBack") {
      if (stopped != 0) {
        delayStopped(500);
      }
      back(500);
    }
    if (text.substring(0, 11) == "engineLeft") {
      if (stopped == 1) {
        delayStopped(500);
        left(500);
        delayStopped(500);
        forward(500);
      } else {
        if (stopped == -1) {
          delayStopped(500);
          left(500);
          delayStopped(500);
          back(500);
        } else {
          if (stopped == 0) {
            left(500);
            delayStopped(500);
          }
        }
      }

    }
    if (text.substring(0, 12) == "engineRight") {
      if (stopped == 1) {
        delayStopped(500);
        right(500);
        delayStopped(500);
        forward(500);
      } else {
        if (stopped == -1) {
          delayStopped(500);
          right(500);
          delayStopped(500);
          back(500);
        } else {
          if (stopped == 0) {
            right(500);
            delayStopped(500);
          }
        }
      }
    }
    if (text.substring(0, 4) == "stop") {
      stopped = 0;
      delayStopped(500);
    }

  }

  delay(500);

}
