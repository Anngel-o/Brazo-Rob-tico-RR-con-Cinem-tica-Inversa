#include <Servo.h>

Servo servo1;
Servo servo2;

String inputString = "";
boolean stringComplete = false;

float targetAngle1 = 90; // Initial angle
float targetAngle2 = 90; // Initial angle

float currentAngle1 = 90;
float currentAngle2 = 90;

const float stepSize = 1; // Adjust for smoothness
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 20; // Update every 20 milliseconds

void setup() {
  Serial.begin(115200);
  servo1.attach(9);
  servo2.attach(10);
  inputString.reserve(200);

  servo1.write(currentAngle1);
  servo2.write(currentAngle2);
}

void loop() {
  // Check for new serial data
  if (stringComplete) {
    parseData(inputString);
    inputString = "";
    stringComplete = false;
  }

  // Update servos at a regular interval
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    updateServos();
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void parseData(String data) {
  data.trim();
  data.replace("[", "");
  data.replace("]", "");
  data.replace("\n", "");
  data.replace("\r", "");

  int commaIndex = data.indexOf(',');
  if (commaIndex == -1) {
    Serial.println("Invalid data received (no comma found)");
    return;
  }

  String angle1Str = data.substring(0, commaIndex);
  String angle2Str = data.substring(commaIndex + 1);

  angle1Str.trim();
  angle2Str.trim();

  float angle1 = angle1Str.toFloat();
  float angle2 = angle2Str.toFloat();

  angle1 = constrain(angle1, 0, 180);
  angle2 = constrain(angle2, 0, 180);

  // Set target angles
  targetAngle1 = angle1;
  targetAngle2 = angle2;
}

void updateServos() {
  // Smoothly move servo1 towards targetAngle1
  if (currentAngle1 < targetAngle1) {
    currentAngle1 += stepSize;
    if (currentAngle1 > targetAngle1) {
      currentAngle1 = targetAngle1;
    }
  } else if (currentAngle1 > targetAngle1) {
    currentAngle1 -= stepSize;
    if (currentAngle1 < targetAngle1) {
      currentAngle1 = targetAngle1;
    }
  }

  // Smoothly move servo2 towards targetAngle2
  if (currentAngle2 < targetAngle2) {
    currentAngle2 += stepSize;
    if (currentAngle2 > targetAngle2) {
      currentAngle2 = targetAngle2;
    }
  } else if (currentAngle2 > targetAngle2) {
    currentAngle2 -= stepSize;
    if (currentAngle2 < targetAngle2) {
      currentAngle2 = targetAngle2;
    }
  }

  // Write the updated angles to the servos
  servo1.write(currentAngle1);
  servo2.write(currentAngle2);
}
