#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Pines I2C configurados en GPIO 18 y 19 para ESP32
#define SDA_PIN 18
#define SCL_PIN 19

// Instancia PCA9685 (Dirección I2C 0x40)
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125 // ~0°
#define SERVOMAX  575 // ~180°
#define SERVO_FREQ 50 // 50 Hz para servos

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Iniciando PWM en las 16 salidas (ESP32)...");

  // Inicializar bus I2C en GPIO 18 y 19
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100 kHz para máxima estabilidad

  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);

  delay(10);
  Serial.println("PCA9685 lista. Transmitiendo a canales 0-15.");
}

void loop() {
  Serial.println("Todas las salidas a 0°");
  setAllServosAngle(0);
  delay(1500);

  Serial.println("Todas las salidas a 90°");
  setAllServosAngle(90);
  delay(1500);

  Serial.println("Todas las salidas a 180°");
  setAllServosAngle(180);
  delay(1500);
}
 
// Función para mapear y actualizar los 16 canales
void setAllServosAngle(uint8_t angle) {
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  
  for (uint8_t ch = 0; ch < 16; ch++) {
    pca.setPWM(ch, 0, pulse);
  }
}