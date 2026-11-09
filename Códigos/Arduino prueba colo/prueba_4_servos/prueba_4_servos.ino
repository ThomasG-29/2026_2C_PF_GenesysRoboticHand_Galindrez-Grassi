// Prueba de 4 servos MG90S directo en el Arduino UNO (sin PCA9685)
// - Señales (cable amarillo/naranja) en D3, D5, D6 y D9.
// - Alimentación de los servos desde el pack de pilas, con GND común al Arduino.
// - Arrancan en 0° y se comandan desde el Monitor Serie (115200 baudios):
//     "90"    -> los 4 servos a 90°
//     "2 45"  -> solo el servo 2 a 45° (servos 1 a 4)

#include <Servo.h>

const uint8_t CANTIDAD_SERVOS = 4;
const uint8_t PINES[CANTIDAD_SERVOS] = { 3, 5, 6, 9 };

// Pulso para 0° y 180° de cada servo. Si alguno se pasa de tope o zumba en las puntas, achicar el suyo.
const int US_MIN[CANTIDAD_SERVOS] = {  600,  600,  600,  600 };
const int US_MAX[CANTIDAD_SERVOS] = { 2400, 2400, 2400, 2400 };

Servo servos[CANTIDAD_SERVOS];

void servoAngulo(uint8_t i, int angulo) {
  servos[i].writeMicroseconds(map(angulo, 0, 180, US_MIN[i], US_MAX[i]));
}

void ayuda() {
  Serial.println("Comandos: \"90\" = los 4 a 90 grados | \"2 45\" = servo 2 a 45 grados (servos 1-4, angulo 0-180)");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);   // anda con o sin fin de línea en el Monitor Serie
  Serial.println("\n=== Prueba 4 servos (D3, D5, D6, D9) ===");

  // Se carga el pulso de 0° antes de attach para que el primer pulso ya sea el 0
  for (uint8_t i = 0; i < CANTIDAD_SERVOS; i++) {
    servos[i].writeMicroseconds(US_MIN[i]);
    servos[i].attach(PINES[i], US_MIN[i], US_MAX[i]);
  }
  Serial.println("Servos en 0.");
  ayuda();
}

void loop() {
  if (!Serial.available()) return;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;

  int espacio = linea.indexOf(' ');
  int servo = 0;   // 0 = todos
  int angulo;
  if (espacio < 0) {
    angulo = linea.toInt();
  } else {
    servo = linea.substring(0, espacio).toInt();
    angulo = linea.substring(espacio + 1).toInt();
    if (servo < 1 || servo > CANTIDAD_SERVOS) {
      Serial.println("Servo invalido.");
      ayuda();
      return;
    }
  }

  if (angulo < 0 || angulo > 180 || (angulo == 0 && linea.charAt(linea.length() - 1) != '0')) {
    Serial.println("Angulo invalido.");
    ayuda();
    return;
  }

  if (servo == 0) {
    for (uint8_t i = 0; i < CANTIDAD_SERVOS; i++) servoAngulo(i, angulo);
    Serial.print("Todos a "); Serial.println(angulo);
  } else {
    servoAngulo(servo - 1, angulo);
    Serial.print("Servo "); Serial.print(servo); Serial.print(" a "); Serial.println(angulo);
  }
}
