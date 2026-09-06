#include <Servo.h>
// ===== CONFIGURACIÓN =====
const int NUM_SENSORES = 5;
const int pinesFlex[NUM_SENSORES]  = {A0, A1, A2, A3, A4};   // Pines de los sensores flex
const int pinesServo[NUM_SENSORES] = {3, 5, 6, 9, 10};       // Pines PWM para los servos
const float VCC = 5.0;
const float R_FIJA = 10000.0;   // Ajustá según tu resistencia fija real
const int ADC_MAX = 1023;
// IMPORTANTE: calibrá estos valores para CADA sensor con el Monitor Serie
// (pueden variar un poco entre sensores aunque sean el mismo modelo)
int flexPlano[NUM_SENSORES]   = {700, 700, 700, 700, 700};
int flexDoblado[NUM_SENSORES] = {900, 900, 900, 900, 900};
Servo servos[NUM_SENSORES];
int lecturaAnterior[NUM_SENSORES];
void setup() {
  Serial.begin(115200);
  delay(1000);
  for (int i = 0; i < NUM_SENSORES; i++) {
    lecturaAnterior[i] = analogRead(pinesFlex[i]);
    servos[i].attach(pinesServo[i]);
    servos[i].write(0);   // Posiciona el servo en 0° al iniciar
  }
  Serial.println("=== Test de 5 Sensores Flex + 5 Servos ===");
}
void loop() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    int lecturaActual = analogRead(pinesFlex[i]);
    // FILTRO: suaviza la señal (90% anterior - 10% nueva)
    int valorSuave = (lecturaAnterior[i] * 0.9) + (lecturaActual * 0.1);
    lecturaAnterior[i] = valorSuave;
    // Calcula resistencia (solo para mostrar en el monitor)
    float voltaje = (valorSuave * VCC) / ADC_MAX;
    float resistenciaFlex = 0;
    if (voltaje > 0) {
      resistenciaFlex = R_FIJA * ((VCC / voltaje) - 1);
    }
    // Mapea el valor suavizado a un ángulo de servo (0-180)
    int valorLimitado = constrain(valorSuave, flexPlano[i], flexDoblado[i]);
    int anguloServo = map(valorLimitado, flexPlano[i], flexDoblado[i], 0, 180);
    servos[i].write(anguloServo);
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(resistenciaFlex, 0);
    Serial.print("Ω (");
    Serial.print(anguloServo);
    Serial.print("°)\t");
  }
  Serial.println();
  delay(30);
}