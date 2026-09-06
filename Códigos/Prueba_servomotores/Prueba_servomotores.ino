#include <Servo.h>

Servo miServo;

const int pinServo = 6;  // Pin PWM del Arduino (9, 10, 11, etc.)

void setup() {
  Serial.begin(9600);
  
  miServo.attach(pinServo);
  
  Serial.println("Iniciando prueba de servo...");
}

void loop() {
  // Barrido de 0° a 180°
  for (int angulo = 0; angulo <= 180; angulo++) {
    miServo.write(angulo);
    Serial.print("Angulo: ");
    Serial.println(angulo);
    delay(15);
  }

  delay(500); // Pausa en el extremo

  // Barrido de vuelta, de 180° a 0°
  for (int angulo = 180; angulo >= 0; angulo--) {
    miServo.write(angulo);
    Serial.print("Angulo: ");
    Serial.println(angulo);
    delay(15);
  }

  delay(500); // Pausa antes de repetir
}