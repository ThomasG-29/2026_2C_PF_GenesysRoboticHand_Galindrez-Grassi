#include <Servo.h>

Servo miServo;

const int pinFlex = A1;
const int pinServo = 2;

// Ajusta estos números con tus valores del Monitor Serie
const int flexPlano = 700;   
const int flexDoblado = 900; 

// Variables para el suavizado (Filtro)
int lecturaAnterior = 0;

void setup() {
  Serial.begin(9600);
  miServo.attach(pinServo);
  lecturaAnterior = analogRead(pinFlex); // Lectura inicial
}

void loop() {
  int lecturaActual = analogRead(pinFlex);
  
  // FILTRO: Suaviza la señal combinando la lectura anterior con la nueva (90% - 10%)
  // Esto elimina las oscilaciones rápidas y el ruido del servo
  int valorSuave = (lecturaAnterior * 0.9) + (lecturaActual * 0.1);
  lecturaAnterior = valorSuave;

  // Restringe y mapea con el valor suavizado
  int valorLimitado = constrain(valorSuave, flexPlano, flexDoblado);
  int anguloServo = map(valorLimitado, flexPlano, flexDoblado, 0, 180);
  
  miServo.write(anguloServo);
  
  Serial.print("Flex Suave: ");
  Serial.print(valorSuave);
  Serial.print("  |  Angulo: ");
  Serial.println(anguloServo);
  
  delay(30); // Pausa óptima para estabilidad del servo
}

