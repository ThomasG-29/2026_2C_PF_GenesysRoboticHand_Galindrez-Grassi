#include <Servo.h>

// Definición de pines
const int pinFlex = A1;   // Entrada analógica del sensor flex
const int pinServo = 9;   // Pin digital PWM para el servomotor

// Creamos el objeto servo
Servo miServo;

// Ajusta estos valores según las lecturas reales de tu sensor flex:
// Puedes ver estos límites en el Monitor Serie manteniendo el sensor recto y doblado.
const int flexMin = 240;  // Lectura analógica cuando el flex está recto/desdoblado
const int flexMax = 400;  // Lectura analógica cuando el flex está doblado al máximo

void setup() {
  Serial.begin(9600);
  
  // Asignamos el pin al servomotor
  miServo.attach(pinServo);
}

void loop() {
  // 1. Leer el valor del sensor flex (0 a 1023)
  int flexLectura = analogRead(pinFlex);

  // 2. Mapear el rango de la lectura analógica al rango de grados del servo (0 a 180)
  int angulo = map(flexLectura, flexMin, flexMax, 0, 180);

  // 3. Limitar el valor del ángulo entre 0 y 180 para evitar saltos o posiciones fuera de rango
  angulo = constrain(angulo, 0, 180);

  // 4. Mover el servomotor al ángulo calculado
  miServo.write(angulo);

  // Muestra los valores por el Monitor Serie para verificar la calibración
  Serial.print("Lectura Flex: ");
  Serial.print(flexLectura);
  Serial.print("  |  Ángulo Servo: ");
  Serial.println(angulo);

  // Pequeña pausa para permitir que el servo alcance la posición
  delay(20);
}
