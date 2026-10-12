#include <Servo.h>

Servo miServo;

const int pinPotenciometro = A0;  // Pin analógico del potenciómetro
const int pinServo = 9;           // Pin digital PWM del servo

int valorPot = 0;
int angulo = 0;

void setup() {
  miServo.attach(pinServo);
  Serial.begin(9600);
}

void loop() {
  valorPot = analogRead(pinPotenciometro);      // Lee 0-1023
  angulo = map(valorPot, 0, 1023, 0, 180);      // Convierte a 0-180°
  
  miServo.write(angulo);
  
  Serial.print("Potenciometro: ");
  Serial.print(valorPot);
  Serial.print(" -> Angulo: ");
  Serial.println(angulo);
  
  delay(15);  // Pequeña pausa para que el servo se estabilice
}