// Prueba de servos en PCA9685 con Arduino UNO
// - Mueve juntos los canales 0 a 3 de 0° a 180° y vuelta, a máxima velocidad. El resto queda apagado.
// - I2C por software (sin librerías) en D5 = SDA y D6 = SCL.
// Monitor serie a 115200 baudios.

const uint8_t PIN_SDA = 5;
const uint8_t PIN_SCL = 6;
const uint8_t DIRECCION = 0x40;

const uint8_t CANTIDAD_SERVOS = 4;   // canales 0 a 3
// Pulso para 0° y 180° de cada canal. El canal 0 va de 700 a 2300 porque con 500/2500 se pasaba de tope.
const int US_MIN[CANTIDAD_SERVOS] = {  700,  500,  500,  500 };
const int US_MAX[CANTIDAD_SERVOS] = { 2300, 2500, 2500, 2500 };

const int ESPERA_EXTREMO_MS = 700;   // tiempo para llegar al otro extremo (bajarlo si llegan antes)

// ---------- I2C por software (open-drain) ----------
void lineaAlta(uint8_t p) { pinMode(p, INPUT_PULLUP); }
void lineaBaja(uint8_t p) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
void esperar() { delayMicroseconds(5); }

void i2cStart() {
  lineaAlta(PIN_SDA); lineaAlta(PIN_SCL); esperar();
  lineaBaja(PIN_SDA); esperar();
  lineaBaja(PIN_SCL); esperar();
}

void i2cStop() {
  lineaBaja(PIN_SDA); esperar();
  lineaAlta(PIN_SCL); esperar();
  lineaAlta(PIN_SDA); esperar();
}

bool i2cEscribir(uint8_t dato) {
  for (int i = 7; i >= 0; i--) {
    if (dato & (1 << i)) lineaAlta(PIN_SDA); else lineaBaja(PIN_SDA);
    esperar();
    lineaAlta(PIN_SCL); esperar();
    lineaBaja(PIN_SCL); esperar();
  }
  lineaAlta(PIN_SDA); esperar();          // soltar SDA para el ACK
  lineaAlta(PIN_SCL); esperar();
  bool ack = (digitalRead(PIN_SDA) == LOW);
  lineaBaja(PIN_SCL); esperar();
  return ack;
}

// ---------- PCA9685 ----------
void pcaEscribir(uint8_t reg, uint8_t valor) {
  i2cStart();
  i2cEscribir(DIRECCION << 1);
  i2cEscribir(reg);
  i2cEscribir(valor);
  i2cStop();
}

void pcaIniciar() {
  pcaEscribir(0x00, 0x10);   // MODE1: sleep para poder cambiar el prescaler
  pcaEscribir(0xFE, 121);    // PRESCALE: 25 MHz / (4096 * 50 Hz) - 1 = 121 -> 50 Hz
  pcaEscribir(0x00, 0x20);   // MODE1: despierto, auto-incremento
  delay(1);
  pcaEscribir(0x00, 0xA0);   // MODE1: restart
  pcaEscribir(0x01, 0x04);   // MODE2: salidas totem-pole
  pcaEscribir(0xFD, 0x10);   // ALL_LED_OFF_H: apaga los 16 canales (el PCA guarda lo anterior si no se le corta la alimentación)
}

// Pone el mismo ángulo en los canales 0..CANTIDAD_SERVOS-1 en una sola transmisión
void servosAngulo(int angulo) {
  i2cStart();
  i2cEscribir(DIRECCION << 1);
  i2cEscribir(0x06);                // LED0_ON_L, luego auto-incremento
  for (uint8_t c = 0; c < CANTIDAD_SERVOS; c++) {
    uint16_t microseg = map(angulo, 0, 180, US_MIN[c], US_MAX[c]);
    uint16_t cuenta = (uint32_t)microseg * 4096 / 20000;
    i2cEscribir(0);
    i2cEscribir(0);
    i2cEscribir(cuenta & 0xFF);
    i2cEscribir(cuenta >> 8);
  }
  i2cStop();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);
  Serial.println("\n=== Prueba servos PCA9685 (canales 0-3) ===");

  lineaAlta(PIN_SDA); lineaAlta(PIN_SCL);
  delay(2);
  while (true) {
    i2cStart();
    bool ack = i2cEscribir(DIRECCION << 1);
    i2cStop();
    if (ack) break;
    Serial.println("No se encontro el PCA9685 en D5/D6. Reintentando...");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(2000);
  }

  pcaIniciar();
  servosAngulo(0);
  Serial.println("PCA9685 OK. Servos en 0.");
  delay(1000);
}

// Salto directo entre extremos: el servo va a su velocidad máxima
void loop() {
  servosAngulo(180);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(ESPERA_EXTREMO_MS);

  servosAngulo(0);
  digitalWrite(LED_BUILTIN, LOW);
  delay(ESPERA_EXTREMO_MS);
}
