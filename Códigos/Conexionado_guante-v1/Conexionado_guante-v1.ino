#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// =====================================================
// ESP32-S3 MINI
// =====================================================

// I2C
#define SDA_PIN 8
#define SCL_PIN 9
 
// =====================================================
// ADS1115
// =====================================================

Adafruit_ADS1115 ads;

// Entrada del ADS1115 donde está conectado SIG del 4067
#define ADS_CHANNEL 0


// =====================================================
// CD74HC4067
// =====================================================

// Pines de selección del multiplexor
#define MUX_S0 10
#define MUX_S1 2
#define MUX_S2 3
#define MUX_S3 4


// =====================================================
// SENSORES FLEX
// =====================================================

#define NUM_FLEX 5

// Canales del CD74HC4067
//
// C0 -> D1
// C1 -> D2
// C2 -> D3
// C3 -> D4
// C4 -> D5

const uint8_t flexChannel[NUM_FLEX] = {
  0,  // D1
  1,  // D2
  2,  // D3
  3,  // D4
  4   // D5
};


// =====================================================
// SELECCIONAR CANAL DEL 4067
// =====================================================

void selectMuxChannel(uint8_t channel)
{
  digitalWrite(MUX_S0, channel & 0x01);
  digitalWrite(MUX_S1, channel & 0x02);
  digitalWrite(MUX_S2, channel & 0x04);
  digitalWrite(MUX_S3, channel & 0x08);

  // Tiempo para que la señal se estabilice
  delayMicroseconds(100);
}


// =====================================================
// LEER SENSOR FLEX
// =====================================================

int16_t readFlex(uint8_t sensor)
{
  // Seleccionar canal correspondiente
  selectMuxChannel(flexChannel[sensor]);

  // Leer mediante ADS1115
  int16_t value = ads.readADC_SingleEnded(ADS_CHANNEL);

  return value;
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("=================================");
  Serial.println("  ESP32-S3 - 5 SENSORES FLEX");
  Serial.println("=================================");


  // ---------------------------------------------------
  // I2C
  // ---------------------------------------------------

  Wire.begin(SDA_PIN, SCL_PIN);


  // ---------------------------------------------------
  // Inicializar ADS1115
  // ---------------------------------------------------

  if (!ads.begin(0x48))
  {
    Serial.println("ERROR: ADS1115 no encontrado.");

    while (1)
    {
      delay(100);
    }
  }

  Serial.println("ADS1115 encontrado.");


  // Ganancia:
  // ±4.096 V
  ads.setGain(GAIN_ONE);


  // ---------------------------------------------------
  // Pines del CD74HC4067
  // ---------------------------------------------------

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);


  // Inicializar multiplexor
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);


  Serial.println("CD74HC4067 configurado.");
  Serial.println("Sistema listo.");
  Serial.println();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  int16_t D1;
  int16_t D2;
  int16_t D3;
  int16_t D4;
  int16_t D5;


  // ---------------------------------------------------
  // Leer los 5 sensores
  // ---------------------------------------------------

  D1 = readFlex(0);
  D2 = readFlex(1);
  D3 = readFlex(2);
  D4 = readFlex(3);
  D5 = readFlex(4);


  // ---------------------------------------------------
  // Mostrar resultados
  // ---------------------------------------------------

  Serial.print("D1: ");
  Serial.print(D1);

  Serial.print("    D2: ");
  Serial.print(D2);

  Serial.print("    D3: ");
  Serial.print(D3);

  Serial.print("    D4: ");
  Serial.print(D4);

  Serial.print("    D5: ");
  Serial.println(D5);


  delay(50);
}