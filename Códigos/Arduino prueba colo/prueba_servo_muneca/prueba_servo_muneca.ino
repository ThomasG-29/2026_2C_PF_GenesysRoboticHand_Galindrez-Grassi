// Prueba servo de la muñeca - Arduino Uno
// Servo GXservo Y3225MG (25 kg)
//
// Al encender, el servo va directo a 90° (centro), para poder ensamblar
// la muñeca con giro hacia ambos lados.
//
// Monitor serie del Arduino IDE a 9600 baudios:
//   - Escribir un ángulo (0 a 180) y Enter -> el servo va a ese ángulo.  Ej: 10, 20, 90
//   - Escribir un valor de 500 a 2500 -> pulso en microsegundos (ajuste fino).  Ej: 1500
//
// ALIMENTACIÓN: un servo de 25 kg puede consumir 2-3 A al arrancar o trabarse.
// El pin 5V del Uno (por USB) no da esa corriente y la placa se puede reiniciar.
// Lo correcto es alimentar el servo con una fuente externa (5-6 V, 3 A o más)
// y unir el GND de la fuente con el GND del Arduino.

#include <Servo.h>

Servo servoMuneca;

const int pinServo = 3;            // Señal PWM del servo
const int anguloInicial = 90;      // Posición de ensamblado

// Rango de pulso del servo: 500 us = 0°, 1500 us = 90° (centro), 2500 us = 180°
const int pulsoMin = 500;
const int pulsoMax = 2500;

void setup() {
  // Se manda la posición antes de attach() para que el primer pulso ya sea 90°
  servoMuneca.write(anguloInicial);
  servoMuneca.attach(pinServo, pulsoMin, pulsoMax);

  Serial.begin(9600);
  Serial.setTimeout(100);  // Funciona con o sin fin de línea en el monitor
  Serial.println("Servo de la muneca listo.");
  Serial.print("Posicion inicial: ");
  Serial.print(anguloInicial);
  Serial.println(" grados (1500 us)");
  Serial.println("Ingresa un angulo (0-180) o un pulso en us (500-2500):");
}

void loop() {
  if (!Serial.available()) return;

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();
  if (entrada.length() == 0) return;

  long valor = entrada.toInt();
  if (valor == 0 && entrada != "0") {
    Serial.print("No es un numero: ");
    Serial.println(entrada);
    return;
  }

  if (valor >= 0 && valor <= 180) {
    servoMuneca.write((int)valor);
    Serial.print("-> Angulo: ");
    Serial.print(valor);
    Serial.print(" grados (");
    Serial.print(servoMuneca.readMicroseconds());
    Serial.println(" us)");
  } else if (valor >= pulsoMin && valor <= pulsoMax) {
    servoMuneca.writeMicroseconds((int)valor);
    Serial.print("-> Pulso: ");
    Serial.print(valor);
    Serial.println(" us");
  } else {
    Serial.print("Fuera de rango: ");
    Serial.print(valor);
    Serial.println("  (angulo 0-180 o pulso 500-2500)");
  }
}
