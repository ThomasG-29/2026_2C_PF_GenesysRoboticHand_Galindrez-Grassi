// ===== TEST DE 5 SENSORES FLEX EN ARDUINO (con filtro + lectura de resistencia) =====

const int NUM_SENSORES = 5;
const int pinesFlex[NUM_SENSORES] = {A0, A1, A2, A3, A4}; // Pines analógicos

const float VCC = 5.0;              // Voltaje de referencia del Arduino
const float R_FIJA = 10000.0;       // Resistencia fija en ohms (ajustá según la tuya)
const int ADC_MAX = 1023;           // Resolución del ADC del Arduino (10 bits)

// Variables para el suavizado (Filtro) - una por cada sensor
int lecturaAnterior[NUM_SENSORES];

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Lectura inicial de cada sensor
  for (int i = 0; i < NUM_SENSORES; i++) {
    lecturaAnterior[i] = analogRead(pinesFlex[i]);
  }

  Serial.println("=== Test de 5 Sensores Flex (Resistencia en Ohms) ===");
}

void loop() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    int lecturaActual = analogRead(pinesFlex[i]);

    // FILTRO: Suaviza la señal combinando la lectura anterior con la nueva (90% - 10%)
    int valorSuave = (lecturaAnterior[i] * 0.9) + (lecturaActual * 0.1);
    lecturaAnterior[i] = valorSuave;

    // Convierte la lectura ADC a voltaje
    float voltaje = (valorSuave * VCC) / ADC_MAX;

    // Calcula la resistencia del sensor flex usando la fórmula del divisor de voltaje
    float resistenciaFlex = 0;
    if (voltaje > 0) {
      resistenciaFlex = R_FIJA * ((VCC / voltaje) - 1);
    }

    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(resistenciaFlex, 0);
    Serial.print(" Ω\t");
  }
  Serial.println();

  delay(100);
}