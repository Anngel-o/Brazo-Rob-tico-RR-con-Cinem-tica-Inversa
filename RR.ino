#include <Servo.h>

Servo servo1;
Servo servo2;

String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(115200);
  servo1.attach(9);  // Ajusta el pin según tu conexión
  servo2.attach(10); // Ajusta el pin según tu conexión
  inputString.reserve(200);
}

void loop() {
  if (stringComplete) {
    parseData(inputString);
    inputString = "";
    stringComplete = false;
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
    Serial.println("Datos inválidos recibidos");
    return;
  }
  String angle1Str = data.substring(0, commaIndex);
  String angle2Str = data.substring(commaIndex + 1);
  float angle1 = angle1Str.toFloat();
  float angle2 = angle2Str.toFloat();

  angle1 = constrain(angle1, 0, 180);
  angle2 = constrain(angle2, 0, 180);

  servo1.write(angle1);
  servo2.write(angle2);

  // Imprimir los ángulos para depuración
  Serial.print("Ángulos recibidos: ");
  Serial.print(angle1);
  Serial.print(", ");
  Serial.println(angle2);
}
