#include <Arduino.h>
#include "bt-remote/receivers/cpp/BtRcReceiver.h"

void setup() {

  int turnPin = 11;
  int leftMotorPin = 10;
  int rightMotorPin = 9;

  int headLightPin = 5;
  int tailLightPin = 3;
  int honkPin = 6;

  int minTurn = 100;
  int maxTurn = 250;

  int minMotor = 133;
  int maxMotor = 240;

  int currentTurn = (minTurn + maxTurn) / 2;
  int speedStop = (minMotor + maxMotor) / 2;

  int tailLightSpeedThreshold = 10;

  int lastTurnWrote = 0;
  int lastSpeedWrote = 0;

  unsigned long lastControlCycle = 0;
  unsigned long lastSendStatus = 0;
  unsigned long lastBlinkCycle = 0;
  bool blinkState = false;

  BtRcReceiver<HardwareSerial> rc(&Serial);
  rc.init("Noco CAR", "1234");

  pinMode(turnPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);

  pinMode(headLightPin, OUTPUT);
  pinMode(tailLightPin, OUTPUT);

  pinMode(honkPin, OUTPUT);

  while(true) {
    rc.read();

    int8_t rawTurn = rc.getX1();
    int8_t rawSpeed = rc.getY1();

    float newTurn = map(rawTurn, -128, 127, minTurn, maxTurn);
    float newSpeed = map(rawSpeed, -128, 127, minMotor, maxMotor);

    unsigned long now = millis();

    if (now - lastBlinkCycle > 500) {
      lastBlinkCycle = now;
      blinkState = !blinkState;
    }

    if (now - lastSendStatus > 50) {
      lastSendStatus = now;

      int sensorValue = analogRead(A0);
      float rawVoltage = (float)sensorValue * (5.0 / 1023.0);
      float adjustedVoltage = rawVoltage * 3;  // I'm using x3 voltage divider before measuring
      char vbuf[5];
      char sbuf[5];
      char tbuf[5];
      char ebuf[5];

      dtostrf(adjustedVoltage, 4, 2, vbuf);
      itoa(rawSpeed, sbuf, 10);
      itoa(rawTurn, tbuf, 10);
      itoa(rc.getChecksumErrorCount(), ebuf, 10);

      rc.send(String("Voltage: ")+vbuf+"V"+"\tSpeed: "+sbuf+"\tTurn: "+tbuf+"\tErrors: "+ebuf);
    }

    if (now - lastControlCycle > 20) {
      lastControlCycle = now;

      currentTurn = currentTurn + (newTurn - currentTurn) / 40;
      float maxDelta = 1;
      float deltaTurn = newTurn - currentTurn;

      if (abs(deltaTurn) > maxDelta) {
        currentTurn += deltaTurn > 0 ? maxDelta : -maxDelta;
      } else {
        currentTurn += deltaTurn;
      }

      if (lastTurnWrote != currentTurn) {
        analogWrite(turnPin, currentTurn);
        lastTurnWrote = currentTurn;
      }

      if (lastSpeedWrote != newSpeed) {
        analogWrite(leftMotorPin, newSpeed);
        analogWrite(rightMotorPin, maxMotor-(newSpeed-minMotor));
        lastSpeedWrote = newSpeed;
      }

      if (rc.getB()) {  // Emergency flasher
        if (blinkState) {
          analogWrite(headLightPin, 50);
          digitalWrite(tailLightPin, HIGH);
        } else {
          digitalWrite(headLightPin, LOW);
          digitalWrite(tailLightPin, LOW);
        }
      } else {
        if (abs(newSpeed - speedStop) > tailLightSpeedThreshold) {
          if (rc.getA())  // If head lights on
            analogWrite(tailLightPin, 50);  // Half-on
          else
            digitalWrite(tailLightPin, LOW);  // Full on
        } else {
          digitalWrite(tailLightPin, HIGH);
        }

        digitalWrite(headLightPin, rc.getA() ? HIGH : LOW);  // Head lights
      }

      if (rc.getE())  // Honk
        analogWrite(honkPin, 128);
      else
        digitalWrite(honkPin, LOW);

    }
  }
}

void loop() {

}
