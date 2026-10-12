#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP32Servo.h>

// =====================================================
// ESP32-S3 MINI
// =====================================================

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_ADS1115 ads;
#define ADS_CHANNEL 0

#define MUX_S0 10
#define MUX_S1 2
#define MUX_S2 3
#define MUX_S3 4

// =====================================================
// 7 SENSORES FLEX
// =====================================================

#define FLEX1_CHANNEL 0
#define FLEX2_CHANNEL 1
#define FLEX3_CHANNEL 2
#define FLEX4_CHANNEL 3
#define FLEX5_CHANNEL 4
#define FLEX6_CHANNEL 5
#define FLEX7_CHANNEL 6

// =====================================================
// 5 POTENCIOMETROS LINEALES
// =====================================================

#define POT1_CHANNEL 7
#define POT2_CHANNEL 8
#define POT3_CHANNEL 9
#define POT4_CHANNEL 10
#define POT5_CHANNEL 11

// El sensor 1 sigue controlando el servo
#define FLEX_CHANNEL FLEX5_CHANNEL

#define SERVO_PIN 1
Servo miServo;

const int16_t FLEX_ESTIRADO = 19500;
const int16_t FLEX_DOBLADO  = 17500;

// =====================================================
// FILTRO
// =====================================================

#define MUESTRAS_MEDIANA 15

const int16_t RANGO_MIN_VALIDO = 16500;
const int16_t RANGO_MAX_VALIDO = 20000;

int16_t ultimoValorValido = FLEX_ESTIRADO;

// --- Suavizado extra sobre el resultado de la mediana ---
// alpha más chico = más suave pero más lento en seguir el cambio real
const float ALPHA = 0.3;
float valorSuavizado = -1; // -1 indica "todavia no inicializado"

// =====================================================
// LIMITADOR DE VELOCIDAD DEL SERVO (slew rate)
// =====================================================

const float PASO_MAXIMO = 4.0;
float anguloActualServo = -1;

// =====================================================
// SELECCIONAR CANAL DEL 4067
// =====================================================

void selectMuxChannel(uint8_t channel)
{
digitalWrite(MUX_S0, channel & 0x01);
digitalWrite(MUX_S1, channel & 0x02);
digitalWrite(MUX_S2, channel & 0x04);
digitalWrite(MUX_S3, channel & 0x08);
delayMicroseconds(100);
}

// =====================================================
// LEER SENSOR FLEX (crudo, a través del mux)
// =====================================================

int16_t leerFlexCrudo()
{
selectMuxChannel(FLEX_CHANNEL);
return ads.readADC_SingleEnded(ADS_CHANNEL);
}

// =====================================================
// LEER SENSOR CON MEDIANA + FILTRO DE RANGO VALIDO
// =====================================================

int16_t leerFlexMediana()
{
int16_t muestras[MUESTRAS_MEDIANA];
int cantidadValidas = 0;

for (int i = 0; i < MUESTRAS_MEDIANA; i++)
{
int16_t lectura = leerFlexCrudo();


if (lectura >= RANGO_MIN_VALIDO && lectura <= RANGO_MAX_VALIDO)
{
  muestras[cantidadValidas] = lectura;
  cantidadValidas++;
}

delayMicroseconds(200);


}

if (cantidadValidas == 0)
{
return ultimoValorValido;
}

for (int i = 1; i < cantidadValidas; i++)
{
int16_t clave = muestras[i];
int j = i - 1;
while (j >= 0 && muestras[j] > clave)
{
muestras[j + 1] = muestras[j];
j--;
}
muestras[j + 1] = clave;
}

int16_t mediana = muestras[cantidadValidas / 2];
ultimoValorValido = mediana;
return mediana;
}

// =====================================================
// LEER LOS OTROS 6 SENSORES FLEX
// SOLO PARA MOSTRARLOS POR SERIAL
// =====================================================

int16_t leerSensorFlex(uint8_t canal)
{
selectMuxChannel(canal);
return ads.readADC_SingleEnded(ADS_CHANNEL);
}

// =====================================================
// LEER LOS 5 POTENCIOMETROS
// SOLO PARA MOSTRARLOS POR SERIAL
// =====================================================

int16_t leerPotenciometro(uint8_t canal)
{
selectMuxChannel(canal);
return ads.readADC_SingleEnded(ADS_CHANNEL);
}

// =====================================================
// APLICAR SUAVIZADO EXTRA (EMA) SOBRE LA MEDIANA
// =====================================================

float suavizarValor(int16_t nuevoValor)
{
if (valorSuavizado < 0)
{
valorSuavizado = nuevoValor;
}
else
{
valorSuavizado = (ALPHA * nuevoValor) + ((1 - ALPHA) * valorSuavizado);
}

return valorSuavizado;
}

// =====================================================
// MOVER SERVO GRADUALMENTE HACIA EL ANGULO OBJETIVO
// =====================================================

void moverServoSuave(int anguloObjetivo)
{
if (anguloActualServo < 0)
{
anguloActualServo = anguloObjetivo;
miServo.write(anguloObjetivo);
return;
}

float diferencia = anguloObjetivo - anguloActualServo;

if (abs(diferencia) <= PASO_MAXIMO)
{
anguloActualServo = anguloObjetivo;
}
else if (diferencia > 0)
{
anguloActualServo += PASO_MAXIMO;
}
else
{
anguloActualServo -= PASO_MAXIMO;
}

miServo.write((int)anguloActualServo);
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
Serial.println("  ESP32-S3 - 7 FLEX + MUX + SERVO");
Serial.println("=================================");

Wire.begin(SDA_PIN, SCL_PIN);

if (!ads.begin(0x48))
{
Serial.println("ERROR: ADS1115 no encontrado.");
while (1) delay(100);
}

Serial.println("ADS1115 encontrado.");
ads.setGain(GAIN_ONE);

pinMode(MUX_S0, OUTPUT);
pinMode(MUX_S1, OUTPUT);
pinMode(MUX_S2, OUTPUT);
pinMode(MUX_S3, OUTPUT);
selectMuxChannel(FLEX_CHANNEL);

Serial.println("CD74HC4067 configurado.");

miServo.attach(SERVO_PIN);

Serial.println("Sistema listo.");
Serial.println();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
// ===================================================
// SENSOR 1
// Sigue funcionando EXACTAMENTE como antes
// y es el único que controla el servo.
// ===================================================

int16_t valorMediana = leerFlexMediana();
float valorFinal = suavizarValor(valorMediana);

int16_t valorLimitado = constrain((int16_t)valorFinal, FLEX_DOBLADO, FLEX_ESTIRADO);
int anguloObjetivo = map(valorLimitado, FLEX_DOBLADO, FLEX_ESTIRADO, 180, 0);

moverServoSuave(anguloObjetivo);

// ===================================================
// SENSORES 2 AL 7
// SOLO SE LEEN Y SE MUESTRAN POR SERIAL
// ===================================================

int16_t flex2 = leerSensorFlex(FLEX2_CHANNEL);
int16_t flex3 = leerSensorFlex(FLEX3_CHANNEL);
int16_t flex4 = leerSensorFlex(FLEX4_CHANNEL);
int16_t flex5 = leerSensorFlex(FLEX5_CHANNEL);
int16_t flex6 = leerSensorFlex(FLEX6_CHANNEL);
int16_t flex7 = leerSensorFlex(FLEX7_CHANNEL);

// ===================================================
// 5 POTENCIOMETROS
// SOLO SE LEEN Y SE MUESTRAN POR SERIAL
// ===================================================

int16_t pot1 = leerPotenciometro(POT1_CHANNEL);
int16_t pot2 = leerPotenciometro(POT2_CHANNEL);
int16_t pot3 = leerPotenciometro(POT3_CHANNEL);
int16_t pot4 = leerPotenciometro(POT4_CHANNEL);
int16_t pot5 = leerPotenciometro(POT5_CHANNEL);

// ===================================================
// SERIAL
// ===================================================

Serial.print("Flex 1: ");
Serial.print(valorMediana);

Serial.print("    Flex 2: ");
Serial.print(flex2);

Serial.print("    Flex 3: ");
Serial.print(flex3);

Serial.print("    Flex 4: ");
Serial.print(flex4);

Serial.print("    Flex 5: ");
Serial.print(flex5);

Serial.print("    Flex 6: ");
Serial.print(flex6);

Serial.print("    Flex 7: ");
Serial.print(flex7);

Serial.print("    Pot 1: ");
Serial.print(pot1);

Serial.print("    Pot 2: ");
Serial.print(pot2);

Serial.print("    Pot 3: ");
Serial.print(pot3);

Serial.print("    Pot 4: ");
Serial.print(pot4);

Serial.print("    Pot 5: ");
Serial.print(pot5);

Serial.print("    Suavizado F1: ");
Serial.print(valorFinal, 1);

Serial.print("    Angulo servo: ");
Serial.println((int)anguloActualServo);

delay(20);
}