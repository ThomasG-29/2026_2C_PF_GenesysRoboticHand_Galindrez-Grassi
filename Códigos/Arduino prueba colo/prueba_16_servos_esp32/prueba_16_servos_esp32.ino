// =====================================================
// PRUEBA DE MECANICA - ESP32 + PCA9685, 16 servos a mano por el monitor serie
// Mismo conexionado que "5. Mano - ESP32(Receptor)": SDA GPIO21, SCL GPIO22,
// PCA9685 en 0x40. Servo N (S1 a S16) = salida N-1 del PCA9685.
// No usa el guante ni toca la calibracion guardada del receptor.
//
// Comandos (monitor serie a 115200, escribir y Enter):
//   2 180      : servo 2 a 180 grados, a su velocidad maxima
//   2 180 30   : servo 2 a 180 grados, a 30 grados por segundo
//   90         : todos a 90 grados (de a uno, para no hacer un pico de corriente)
//   0 90 30    : todos a 90 grados, juntos, a 30 grados por segundo
//   h          : todos a HOME (la misma posicion que usa el receptor al encender)
//   x /0 x 2    : soltar todos / el servo 2 (sin señal, sin fuerza: se mueve con la mano)
//   s          : ver el angulo de cada servo
// =====================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA 21
#define I2C_SCL 22

// Pulso para 0 y 180 grados de cada modelo (los mismos que usa el receptor)
#define MG90S_DEDOS_MIN_US 400
#define MG90S_DEDOS_MAX_US 2600
#define DS3225_MIN_US 500
#define DS3225_MAX_US 2500
#define MG92B_MIN_US  500
#define MG92B_MAX_US  2500

#define CANT_SERVOS 16
#define PERIODO_MS  10    // actualizacion de los movimientos con velocidad
#define PAUSA_DE_A_UNO_MS 150

typedef struct {
  const char *nombre;
  uint16_t minUs, maxUs;
  uint8_t  anguloHome;
} ServoPrueba;

// Indice = salida del PCA9685 (segun el receptor y Conexionados/Circuito_mano_robot)
const ServoPrueba SERVOS[CANT_SERVOS] = {
  {"S1 MG90S flex 1",              MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S2 MG90S flex 2",              MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S3 MG90S flex 3",              MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S4 MG90S flex 4",              MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S5 MG90S flex 5",              MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S6 MG90S nudillo pulgar",      MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S7 MG90S nudillo indice",      MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S8 MG90S nudillo medio",       MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S9 MG90S nudillo anular",      MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S10 MG90S nudillo menique",    MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,  0},
  {"S11 DS3225 muneca adelante",   DS3225_MIN_US,      DS3225_MAX_US,      10},
  {"S12 DS3225 muneca lateral",    DS3225_MIN_US,      DS3225_MAX_US,      90},
  {"S13 MG92B apertura pulgar",    MG92B_MIN_US,       MG92B_MAX_US,        0},
  {"S14 MG92B apertura indice",    MG92B_MIN_US,       MG92B_MAX_US,        0},
  {"S15 MG92B apertura anular",    MG92B_MIN_US,       MG92B_MAX_US,        0},
  {"S16 MG92B apertura menique",   MG92B_MIN_US,       MG92B_MAX_US,        0},
};

Adafruit_PWMServoDriver pca(0x40);

float angulo[CANT_SERVOS];      // ultimo angulo enviado
float objetivo[CANT_SERVOS];
float velocidad[CANT_SERVOS];   // grados por segundo (0 = sin movimiento en curso)
bool  suelto[CANT_SERVOS];

void escribirServo(int i, float a) {
  const ServoPrueba &s = SERVOS[i];
  float us = s.minUs + a * (s.maxUs - s.minUs) / 180.0f;
  pca.setPWM(i, 0, (uint16_t)lroundf(us * 4096.0f / 20000.0f));
  angulo[i] = a;
  suelto[i] = false;
}

void soltarServo(int i) {
  pca.setPWM(i, 0, 4096);   // 4096 = salida siempre apagada
  velocidad[i] = 0;
  suelto[i] = true;
}

// vel = 0: salto directo (velocidad maxima del servo)
void moverServo(int i, int a, float vel) {
  if (vel <= 0 || suelto[i]) {   // suelto: no se sabe donde esta, va directo
    velocidad[i] = 0;
    escribirServo(i, a);
  } else {
    objetivo[i] = a;
    velocidad[i] = vel;
  }
}

void actualizarMovimientos() {
  static uint32_t tAnterior = millis();
  uint32_t ahora = millis();
  if (ahora - tAnterior < PERIODO_MS) return;
  float dt = (ahora - tAnterior) / 1000.0f;
  tAnterior = ahora;

  for (int i = 0; i < CANT_SERVOS; i++) {
    if (velocidad[i] <= 0) continue;
    float dif = objetivo[i] - angulo[i];
    float paso = velocidad[i] * dt;
    if (fabsf(dif) <= paso) {
      escribirServo(i, objetivo[i]);
      velocidad[i] = 0;
    } else {
      escribirServo(i, angulo[i] + (dif > 0 ? paso : -paso));
    }
  }
}

void todosDeAUno(bool home, int a) {
  for (int i = 0; i < CANT_SERVOS; i++) {
    velocidad[i] = 0;
    escribirServo(i, home ? SERVOS[i].anguloHome : a);
    delay(PAUSA_DE_A_UNO_MS);
  }
}

void ayuda() {
  Serial.println("Comandos:");
  Serial.println("  2 180     servo 2 a 180 grados (servos 1-16, angulo 0-180)");
  Serial.println("  2 180 30  lo mismo a 30 grados por segundo");
  Serial.println("  90        todos a 90 (de a uno)   |  0 90 30  todos juntos a 30 grados/s");
  Serial.println("  h         todos a HOME            |  x / x 2  soltar todos / el servo 2");
  Serial.println("  s         ver el angulo de cada servo");
}

void mostrarEstado() {
  for (int i = 0; i < CANT_SERVOS; i++) {
    Serial.printf("  %-30s ", SERVOS[i].nombre);
    if (suelto[i]) Serial.println("suelto");
    else Serial.printf("%5.1f%s\n", angulo[i], velocidad[i] > 0 ? " (moviendose)" : "");
  }
}

void procesarComando(String linea) {
  linea.trim();
  linea.toLowerCase();
  if (linea.length() == 0) return;

  if (linea == "h") {
    todosDeAUno(true, 0);
    Serial.println("Todos en HOME");
    return;
  }
  if (linea == "s") { mostrarEstado(); return; }
  if (linea.charAt(0) == 'x') {
    int n = 0;
    if (linea.length() > 1 && (sscanf(linea.c_str() + 1, "%d", &n) != 1 || n < 1 || n > CANT_SERVOS)) {
      Serial.println("Servo invalido."); ayuda(); return;
    }
    if (n == 0) { for (int i = 0; i < CANT_SERVOS; i++) soltarServo(i); Serial.println("Todos sueltos"); }
    else { soltarServo(n - 1); Serial.printf("%s suelto\n", SERVOS[n - 1].nombre); }
    return;
  }

  int n1, n2, n3;
  char sobra;
  int cant = sscanf(linea.c_str(), "%d %d %d %c", &n1, &n2, &n3, &sobra);
  int servo, a;
  float vel = 0;
  bool deAUno = false;
  if (cant == 1)      { servo = 0;  a = n1; deAUno = true; }
  else if (cant == 2) { servo = n1; a = n2; }
  else if (cant == 3) { servo = n1; a = n2; vel = n3; }
  else { Serial.println("Comando invalido."); ayuda(); return; }

  if (servo < 0 || servo > CANT_SERVOS) { Serial.println("Servo invalido (1 a 16, 0 = todos)."); return; }
  if (a < 0 || a > 180)                 { Serial.println("Angulo invalido (0 a 180)."); return; }
  if (cant == 3 && vel <= 0)            { Serial.println("Velocidad invalida (grados por segundo, mayor a 0)."); return; }

  if (servo == 0 && (deAUno || vel == 0)) {
    todosDeAUno(false, a);
    Serial.printf("Todos a %d\n", a);
  } else if (servo == 0) {
    for (int i = 0; i < CANT_SERVOS; i++) moverServo(i, a, vel);
    Serial.printf("Todos a %d, a %.0f grados/s\n", a, vel);
  } else {
    moverServo(servo - 1, a, vel);
    Serial.printf("%s a %d", SERVOS[servo - 1].nombre, a);
    if (vel > 0) Serial.printf(", a %.0f grados/s", vel);
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);   // anda con o sin fin de linea en el monitor serie
  delay(500);
  Serial.println("\n=== PRUEBA DE MECANICA: 16 SERVOS (ESP32 + PCA9685) ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  pca.begin();
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() != 0)
    Serial.println("ATENCION: el PCA9685 NO RESPONDE por I2C: revisar SDA (GPIO21), SCL (GPIO22), VCC y GND");
  pca.setPWMFreq(50);
  delay(10);

  // HOME de a uno, igual que el receptor (el primer movimiento es a velocidad maxima)
  todosDeAUno(true, 0);
  Serial.println("Servos en HOME.");
  ayuda();
}

void loop() {
  if (Serial.available()) procesarComando(Serial.readStringUntil('\n'));
  actualizarMovimientos();
}
