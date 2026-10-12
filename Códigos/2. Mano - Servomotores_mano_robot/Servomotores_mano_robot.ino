#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// =====================================================
// ASIGNACIÓN DE CANALES (0 a 15 del PCA9685)
// =====================================================

// MG92B Tower Pro -> canales 0 a 3
const int CANALES_MG92B[] = {0, 1, 2, 3};
const int CANT_MG92B = 4;

// MG90S -> canales 4 a 13
const int CANALES_MG90S[] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const int CANT_MG90S = 10;

// DS3225 -> canales 14 y 15
const int CANALES_DS3225[] = {14, 15};
const int CANT_DS3225 = 2;

// =====================================================
// RANGOS DE PULSO POR MODELO (en microsegundos)
// =====================================================

const int MG92B_MIN_US  = 500;
const int MG92B_MAX_US  = 2500;

const int MG90S_MIN_US  = 500;
const int MG90S_MAX_US  = 2400;

const int DS3225_MIN_US = 500;
const int DS3225_MAX_US = 2500;

// =====================================================
// CONVERSIÓN DE MICROSEGUNDOS A "TICKS" DEL PCA9685
// (a 50Hz, el período es de 20000us dividido en 4096 pasos)
// =====================================================

int usAticks(int microsegundos) {
  return (int)((long)microsegundos * 4096L / 20000L);
}

// =====================================================
// MOVER UN SERVO A UN ÁNGULO DADO, SEGÚN SU RANGO PROPIO
// =====================================================

void moverServo(int canal, int angulo, int pulsoMinUs, int pulsoMaxUs) {
  angulo = constrain(angulo, 0, 180);
  int pulsoUs = map(angulo, 0, 180, pulsoMinUs, pulsoMaxUs);
  pca9685.setPWM(canal, 0, usAticks(pulsoUs));
}

// =====================================================
// MOVER TODOS LOS SERVOS DE UN GRUPO AL MISMO ÁNGULO
// =====================================================

void moverGrupo(const int canales[], int cantidad, int angulo, int minUs, int maxUs) {
  for (int i = 0; i < cantidad; i++) {
    moverServo(canales[i], angulo, minUs, maxUs);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();  // SDA=21, SCL=22 (pines por defecto del ESP32 DevKit)

  pca9685.begin();
  pca9685.setPWMFreq(50);  // Todos los servos estándar trabajan a 50Hz

  delay(10);

  // Cero: lleva los servos a 0° de a uno, para no pedirle a la fuente
  // el pico de corriente de los 16 arrancando juntos
  Serial.println("Llevando servos a 0°...");
  for (int i = 0; i < CANT_MG92B; i++) {
    moverServo(CANALES_MG92B[i], 0, MG92B_MIN_US, MG92B_MAX_US);
    delay(150);
  }
  for (int i = 0; i < CANT_MG90S; i++) {
    moverServo(CANALES_MG90S[i], 0, MG90S_MIN_US, MG90S_MAX_US);
    delay(150);
  }
  for (int i = 0; i < CANT_DS3225; i++) {
    moverServo(CANALES_DS3225[i], 0, DS3225_MIN_US, DS3225_MAX_US);
    delay(150);
  }

  // Se quedan quietos en 0° antes de arrancar la secuencia
  Serial.println("16 servos en 0°. La secuencia arranca en 3 s...");
  delay(3000);
}

// =====================================================
// LOOP: barrido conjunto de 0° a 180° y vuelta
// =====================================================

void loop() {
  // Barrido de 0° a 180°
  for (int angulo = 0; angulo <= 180; angulo++) {
    moverGrupo(CANALES_MG92B, CANT_MG92B, angulo, MG92B_MIN_US, MG92B_MAX_US);
    moverGrupo(CANALES_MG90S, CANT_MG90S, angulo, MG90S_MIN_US, MG90S_MAX_US);
    moverGrupo(CANALES_DS3225, CANT_DS3225, angulo, DS3225_MIN_US, DS3225_MAX_US);

    Serial.print("Angulo: ");
    Serial.println(angulo);
    delay(15);
  }

  delay(500);

  // Barrido de vuelta, de 180° a 0°
  for (int angulo = 180; angulo >= 0; angulo--) {
    moverGrupo(CANALES_MG92B, CANT_MG92B, angulo, MG92B_MIN_US, MG92B_MAX_US);
    moverGrupo(CANALES_MG90S, CANT_MG90S, angulo, MG90S_MIN_US, MG90S_MAX_US);
    moverGrupo(CANALES_DS3225, CANT_DS3225, angulo, DS3225_MIN_US, DS3225_MAX_US);

    Serial.print("Angulo: ");
    Serial.println(angulo);
    delay(15);
  }

  delay(500);
}