// Pulgar: 4 servos MG90S directo en el Arduino UNO, con modo automático y manual
// (basado en prueba_4_servos)
// - Señales (cable amarillo/naranja): motor 1 en D3, motor 2 en D5, motor 3 en D6, motor 4 en D9.
// - Alimentación de los servos desde el pack de pilas, con GND común al Arduino.
// - Arranca en modo MANUAL con los servos en 0°.
// Comandos por el Monitor Serie (115200 baudios):
//   A       -> modo automático: arranca la secuencia (se hace 4 veces, cada vez más rápido, la última a velocidad máxima, y se detiene)
//   I       -> modo automático: vuelve a arrancar la secuencia
//   M       -> modo manual (la secuencia se frena donde esté)
//   P       -> PARADA DE EMERGENCIA: los servos se quedan donde están y pasa a manual
//   En modo manual:
//     "90"    -> los 4 servos a 90°
//     "2 45"  -> solo el servo 2 a 45° (servos 1 a 4)

#include <Servo.h>

const uint8_t CANTIDAD_SERVOS = 4;
const uint8_t PINES[CANTIDAD_SERVOS] = { 3, 5, 6, 9 };

// Pulso para 0° y 180° de cada servo. Si alguno se pasa de tope o zumba en las puntas, achicar el suyo.
const int US_MIN[CANTIDAD_SERVOS] = {  600,  600,  600,  600 };
const int US_MAX[CANTIDAD_SERVOS] = { 2400, 2400, 2400, 2400 };

// ---------- Secuencia automática ----------
const int8_t MANTIENE = -1;   // el motor se queda donde está en ese paso
const uint8_t CANTIDAD_PASOS = 5;
const int8_t SECUENCIA[CANTIDAD_PASOS][CANTIDAD_SERVOS] = {
  // motor 1   motor 2   motor 3   motor 4
  {     0,        0,        0,        0     },   // paso 1: todos a 0
  {     0,       50,       50,   MANTIENE   },   // paso 2: 1 a 0, 2 y 3 a 50
  {    90,       90,  MANTIENE,  MANTIENE   },   // paso 3: 1 y 2 a 90, 3 mantiene
  {    50,       50,  MANTIENE,  MANTIENE   },   // paso 4: 1 y 2 a 50, 3 mantiene
  {     0,        0,        0,   MANTIENE   },   // paso 5: 1, 2 y 3 a 0
};
const uint8_t REPETICIONES = 4;
// Grados por segundo en cada repetición: cada ciclo va más rápido que el anterior.
// VELOCIDAD_MAXIMA = salto directo al ángulo: el servo va a su velocidad máxima (P no lo frena a mitad de camino)
const float VELOCIDAD_MAXIMA = 0;
const float VELOCIDADES[REPETICIONES] = { 40.0, 70.0, 100.0, VELOCIDAD_MAXIMA };
const unsigned long PAUSA_PASO_MS = 1000;    // espera al terminar cada paso
const unsigned long PERIODO_MS = 10;         // actualización del movimiento

Servo servos[CANTIDAD_SERVOS];
float angulo[CANTIDAD_SERVOS];     // último ángulo enviado
float objetivo[CANTIDAD_SERVOS];

bool modoAuto = false;
bool secuenciaCorriendo = false;
uint8_t paso = 0;
uint8_t repeticion = 0;
bool esperando = false;            // terminó el movimiento del paso, haciendo la pausa
unsigned long tFinPaso = 0;

void servoAngulo(uint8_t i, float a) {
  angulo[i] = a;
  servos[i].writeMicroseconds(US_MIN[i] + a * (US_MAX[i] - US_MIN[i]) / 180.0);
}

void ayuda() {
  Serial.println("Comandos: A = auto | I = reiniciar secuencia | M = manual | P = PARADA DE EMERGENCIA");
  Serial.println("  Manual: \"90\" = los 4 a 90 grados | \"2 45\" = servo 2 a 45 grados (servos 1-4, angulo 0-180)");
}

// ---------- Modo automático ----------
void iniciarPaso() {
  for (uint8_t i = 0; i < CANTIDAD_SERVOS; i++) {
    if (SECUENCIA[paso][i] != MANTIENE) objetivo[i] = SECUENCIA[paso][i];
  }
  esperando = false;
  Serial.print("Ciclo "); Serial.print(repeticion + 1); Serial.print("/"); Serial.print(REPETICIONES);
  Serial.print(" - Paso "); Serial.print(paso + 1);
  if (VELOCIDADES[repeticion] == VELOCIDAD_MAXIMA) {
    Serial.println(" (velocidad maxima)");
  } else {
    Serial.print(" ("); Serial.print(VELOCIDADES[repeticion], 0); Serial.println(" grados/s)");
  }
}

void iniciarSecuencia() {
  modoAuto = true;
  secuenciaCorriendo = true;
  paso = 0;
  repeticion = 0;
  Serial.println(">>> Modo AUTO: arranca la secuencia");
  iniciarPaso();
}

// Frena todo donde está (la secuencia y cualquier movimiento a medio camino)
void detener() {
  secuenciaCorriendo = false;
  for (uint8_t i = 0; i < CANTIDAD_SERVOS; i++) objetivo[i] = angulo[i];
}

void actualizarAuto() {
  static unsigned long tAnterior = millis();
  unsigned long ahora = millis();
  if (ahora - tAnterior < PERIODO_MS) return;
  unsigned long dt = ahora - tAnterior;
  tAnterior = ahora;
  if (!secuenciaCorriendo) return;
  float pasoGrados = VELOCIDADES[repeticion] == VELOCIDAD_MAXIMA ? 180.0 : VELOCIDADES[repeticion] * dt / 1000.0;

  bool llegaron = true;
  for (uint8_t i = 0; i < CANTIDAD_SERVOS; i++) {
    float dif = objetivo[i] - angulo[i];
    if (dif == 0) continue;
    llegaron = false;
    if (fabs(dif) <= pasoGrados) servoAngulo(i, objetivo[i]);
    else servoAngulo(i, angulo[i] + (dif > 0 ? pasoGrados : -pasoGrados));
  }
  if (!llegaron) return;

  if (!esperando) {
    esperando = true;
    tFinPaso = ahora;
    return;
  }
  if (ahora - tFinPaso < PAUSA_PASO_MS) return;

  paso++;
  if (paso >= CANTIDAD_PASOS) {
    paso = 0;
    repeticion++;
    if (repeticion >= REPETICIONES) {
      secuenciaCorriendo = false;
      Serial.println(">>> Secuencia terminada. I = repetir | M = manual");
      return;
    }
  }
  iniciarPaso();
}

// ---------- Modo manual (igual que prueba_4_servos) ----------
void comandoManual(String linea) {
  int espacio = linea.indexOf(' ');
  int servo = 0;   // 0 = todos
  int a;
  if (espacio < 0) {
    a = linea.toInt();
  } else {
    servo = linea.substring(0, espacio).toInt();
    a = linea.substring(espacio + 1).toInt();
    if (servo < 1 || servo > CANTIDAD_SERVOS) {
      Serial.println("Servo invalido.");
      ayuda();
      return;
    }
  }

  if (a < 0 || a > 180 || (a == 0 && linea.charAt(linea.length() - 1) != '0')) {
    Serial.println("Comando invalido.");
    ayuda();
    return;
  }

  if (servo == 0) {
    for (uint8_t i = 0; i < CANTIDAD_SERVOS; i++) { servoAngulo(i, a); objetivo[i] = a; }
    Serial.print("Todos a "); Serial.println(a);
  } else {
    servoAngulo(servo - 1, a);
    objetivo[servo - 1] = a;
    Serial.print("Servo "); Serial.print(servo); Serial.print(" a "); Serial.println(a);
  }
}

void procesarComando(String linea) {
  linea.trim();
  linea.toUpperCase();
  if (linea.length() == 0) return;

  if (linea == "P") {
    detener();
    modoAuto = false;
    Serial.println("!!! PARADA DE EMERGENCIA: servos detenidos donde estaban. Modo MANUAL");
    return;
  }
  if (linea == "A" || linea == "I") { iniciarSecuencia(); return; }
  if (linea == "M") {
    detener();
    modoAuto = false;
    Serial.println(">>> Modo MANUAL");
    return;
  }

  if (modoAuto) {
    Serial.println("En modo AUTO: M o P para pasar a manual, I para repetir la secuencia.");
    return;
  }
  comandoManual(linea);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);   // anda con o sin fin de línea en el Monitor Serie
  Serial.println("\n=== Pulgar auto - 4 servos (D3, D5, D6, D9) ===");

  // Se carga el pulso de 0° antes de attach para que el primer pulso ya sea el 0
  for (uint8_t i = 0; i < CANTIDAD_SERVOS; i++) {
    angulo[i] = objetivo[i] = 0;
    servos[i].writeMicroseconds(US_MIN[i]);
    servos[i].attach(PINES[i], US_MIN[i], US_MAX[i]);
  }
  Serial.println("Servos en 0. Modo MANUAL.");
  ayuda();
}

void loop() {
  if (Serial.available()) procesarComando(Serial.readStringUntil('\n'));
  actualizarAuto();
}
