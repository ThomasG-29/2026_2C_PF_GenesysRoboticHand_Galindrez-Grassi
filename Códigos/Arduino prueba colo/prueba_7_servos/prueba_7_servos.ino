// Prueba de 7 servos MG90S directo en el Arduino UNO (sin PCA9685)
// (ampliación de prueba_4_servos)
// - Señales (cable amarillo/naranja): servo 1 en D3, 2 en D5, 3 en D6, 4 en D9,
//   5 en D10, 6 en D11 y 7 en D12. D0 y D1 no se usan: son el puerto serie (USB).
// - Alimentación de los servos desde el pack de pilas, con GND común al Arduino.
// - Arrancan en 0° y se comandan desde el Monitor Serie (115200 baudios):
//     "90"              -> los 7 servos a 90°
//     "2 45"            -> solo el servo 2 a 45° (servos 1 a 7)
//     "2 20 ; 3 50"     -> servo 2 a 20° y servo 3 a 50°, los dos al mismo tiempo
//                          (se pueden encadenar hasta los 7 servos separados por ";")

#include <Servo.h>

const uint8_t CANTIDAD_SERVOS = 7;
const uint8_t PINES[CANTIDAD_SERVOS] = { 3, 5, 6, 9, 10, 11, 12 };

// Pulso para 0° y 180° de cada servo. Si alguno se pasa de tope o zumba en las puntas, achicar el suyo.
const int US_MIN[CANTIDAD_SERVOS] = {  600,  600,  600,  600,  600,  600,  600 };
const int US_MAX[CANTIDAD_SERVOS] = { 2400, 2400, 2400, 2400, 2400, 2400, 2400 };

const uint8_t MAX_NUMEROS = 2 * CANTIDAD_SERVOS;   // como mucho un par servo-ángulo por servo

Servo servos[CANTIDAD_SERVOS];

void servoAngulo(uint8_t i, int angulo) {
  servos[i].writeMicroseconds(map(angulo, 0, 180, US_MIN[i], US_MAX[i]));
}

void ayuda() {
  Serial.println("Comandos: \"90\" = los 7 a 90 grados | \"2 45\" = servo 2 a 45 grados (servos 1-7, angulo 0-180)");
  Serial.println("          \"2 20 ; 3 50\" = servo 2 a 20 y servo 3 a 50 al mismo tiempo (hasta los 7 separados por ;)");
}

// Saca los números de la línea. Espacios, ";" y "," son separadores; cualquier otra cosa es un error.
// Devuelve la cantidad de números, o -1 si la línea no es válida.
int leerNumeros(const String &linea, int numeros[]) {
  int cantidad = 0;
  bool enNumero = false;
  for (unsigned int k = 0; k < linea.length(); k++) {
    char c = linea.charAt(k);
    if (isDigit(c)) {
      if (!enNumero) {
        if (cantidad == MAX_NUMEROS) return -1;
        numeros[cantidad++] = 0;
        enNumero = true;
      }
      numeros[cantidad - 1] = numeros[cantidad - 1] * 10 + (c - '0');
      if (numeros[cantidad - 1] > 999) return -1;
    } else if (c == ' ' || c == ';' || c == ',' || c == '\t') {
      enNumero = false;
    } else {
      return -1;
    }
  }
  return cantidad;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);   // anda con o sin fin de línea en el Monitor Serie
  Serial.println("\n=== Prueba 7 servos (D3, D5, D6, D9, D10, D11, D12) ===");

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

  int numeros[MAX_NUMEROS];
  int cantidad = leerNumeros(linea, numeros);
  if (cantidad < 1 || (cantidad > 1 && cantidad % 2 != 0)) {
    Serial.println("Comando invalido.");
    ayuda();
    return;
  }

  // Un solo número: todos los servos a ese ángulo
  if (cantidad == 1) {
    if (numeros[0] > 180) { Serial.println("Angulo invalido."); ayuda(); return; }
    for (uint8_t i = 0; i < CANTIDAD_SERVOS; i++) servoAngulo(i, numeros[0]);
    Serial.print("Todos a "); Serial.println(numeros[0]);
    return;
  }

  // Pares servo-ángulo: primero se revisan todos y después se mueven juntos
  for (int k = 0; k < cantidad; k += 2) {
    if (numeros[k] < 1 || numeros[k] > CANTIDAD_SERVOS) {
      Serial.print("Servo invalido: "); Serial.println(numeros[k]); ayuda(); return;
    }
    if (numeros[k + 1] > 180) {
      Serial.print("Angulo invalido: "); Serial.println(numeros[k + 1]); ayuda(); return;
    }
  }
  for (int k = 0; k < cantidad; k += 2) servoAngulo(numeros[k] - 1, numeros[k + 1]);

  for (int k = 0; k < cantidad; k += 2) {
    if (k > 0) Serial.print(" | ");
    Serial.print("Servo "); Serial.print(numeros[k]); Serial.print(" a "); Serial.print(numeros[k + 1]);
  }
  Serial.println();
}
