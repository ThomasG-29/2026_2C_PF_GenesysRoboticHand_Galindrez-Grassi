#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// =====================================================
// ESP32-C3 MINI
// =====================================================

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_ADS1115 ads;
#define ADS_CHANNEL 0        // SIG del multiplexor 1 -> A0 del ADS1115
#define ADS_CHANNEL_MUX2 1   // SIG del multiplexor 2 -> A1 del ADS1115

#define MUX_S0 10
#define MUX_S1 2
#define MUX_S2 3
#define MUX_S3 4
// Los pines S0-S3 se comparten entre ambos multiplexores,
// ya que seleccionan el mismo canal en los dos al mismo tiempo.
// El EN de ambos multiplexores va directo a GND.

// =====================================================
// FLEX 1 (multiplexor 2, canal 0)
// =====================================================

#define FLEX1_CHANNEL_MUX2 0

// =====================================================
// 6 SENSORES FLEX RESTANTES (multiplexor 1, sin cambios)
// =====================================================

#define FLEX2_CHANNEL 1
#define FLEX3_CHANNEL 2
#define FLEX4_CHANNEL 3
#define FLEX5_CHANNEL 4
#define FLEX6_CHANNEL 5
#define FLEX7_CHANNEL 6

// =====================================================
// 5 POTENCIOMETROS LINEALES (multiplexor 1, sin cambios)
// =====================================================

#define POT1_CHANNEL 7
#define POT2_CHANNEL 8
#define POT3_CHANNEL 9
#define POT4_CHANNEL 10
#define POT5_CHANNEL 11

// =====================================================
// 5 POTENCIOMETROS NUEVOS (multiplexor 2, canales 1 a 5
// ya que el canal 0 lo ocupa el Flex 1)
// =====================================================

#define POT6_CHANNEL 1
#define POT7_CHANNEL 2
#define POT8_CHANNEL 3
#define POT9_CHANNEL 4
#define POT10_CHANNEL 5

// =====================================================
// SELECCIONAR CANAL (afecta a AMBOS multiplexores a la vez,
// ya que S0-S3 estan compartidos entre los dos)
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
// LEER FLEX 1 (multiplexor 2, canal 0) - igual que los demas
// =====================================================

int16_t leerFlex1(uint8_t canal)
{
selectMuxChannel(canal);
return ads.readADC_SingleEnded(ADS_CHANNEL_MUX2);
}

// =====================================================
// LEER LOS OTROS 6 SENSORES FLEX (multiplexor 1)
// =====================================================

int16_t leerSensorFlex(uint8_t canal)
{
selectMuxChannel(canal);
return ads.readADC_SingleEnded(ADS_CHANNEL);
}

// =====================================================
// LEER LOS 5 POTENCIOMETROS (multiplexor 1)
// =====================================================

int16_t leerPotenciometro(uint8_t canal)
{
selectMuxChannel(canal);
return ads.readADC_SingleEnded(ADS_CHANNEL);
}

// =====================================================
// LEER LOS 5 POTENCIOMETROS NUEVOS (multiplexor 2)
// =====================================================

int16_t leerPotenciometro2(uint8_t canal)
{
selectMuxChannel(canal);
return ads.readADC_SingleEnded(ADS_CHANNEL_MUX2);
}

// =====================================================
// TABLA EN PANTALLA: encabezado fijo arriba de la terminal
// y debajo los valores, que se actualizan en el lugar.
// Usa codigos ANSI (requiere monitor_filters = direct).
// =====================================================

#define ESC "\x1B"

void dibujarTabla(const int16_t valores[17])
{
char linea[128];
int pos = 0;
for (int i = 0; i < 17; i++)
{
if (i == 7) pos += snprintf(linea + pos, sizeof(linea) - pos, " |");
pos += snprintf(linea + pos, sizeof(linea) - pos, "%6d", valores[i]);
}

// Todo en un solo envio para que no parpadee:
// cursor al inicio + encabezado + valores + borrar lo que quede abajo
char pantalla[400];
snprintf(pantalla, sizeof(pantalla),
         ESC "[H"
         "    F1    F2    F3    F4    F5    F6    F7 |    P1    P2    P3    P4    P5    P6    P7    P8    P9   P10" ESC "[K\r\n"
         "-------------------------------------------+------------------------------------------------------------" ESC "[K\r\n"
         "%s" ESC "[K\r\n"
         ESC "[J",
         linea);
Serial.print(pantalla);
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
Serial.println("  ESP32-C3 - 7 FLEX + 10 POT + 2 MUX (sin servo)");
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
selectMuxChannel(FLEX1_CHANNEL_MUX2);

Serial.println("CD74HC4067 (x2) configurados.");

Serial.println("Sistema listo.");
delay(1000);
Serial.print(ESC "[2J");  // limpiar la pantalla una vez antes de la tabla
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
// ===================================================
// FLEX 1 (multiplexor 2, canal 0) - lectura simple
// ===================================================

int16_t flex1 = leerFlex1(FLEX1_CHANNEL_MUX2);

// ===================================================
// SENSORES 2 AL 7 (multiplexor 1)
// ===================================================

int16_t flex2 = leerSensorFlex(FLEX2_CHANNEL);
int16_t flex3 = leerSensorFlex(FLEX3_CHANNEL);
int16_t flex4 = leerSensorFlex(FLEX4_CHANNEL);
int16_t flex5 = leerSensorFlex(FLEX5_CHANNEL);
int16_t flex6 = leerSensorFlex(FLEX6_CHANNEL);
int16_t flex7 = leerSensorFlex(FLEX7_CHANNEL);

// ===================================================
// 5 POTENCIOMETROS (multiplexor 1)
// ===================================================

int16_t pot1 = leerPotenciometro(POT1_CHANNEL);
int16_t pot2 = leerPotenciometro(POT2_CHANNEL);
int16_t pot3 = leerPotenciometro(POT3_CHANNEL);
int16_t pot4 = leerPotenciometro(POT4_CHANNEL);
int16_t pot5 = leerPotenciometro(POT5_CHANNEL);

// ===================================================
// 5 POTENCIOMETROS NUEVOS (multiplexor 2, canales 1-5)
// ===================================================

int16_t pot6  = leerPotenciometro2(POT6_CHANNEL);
int16_t pot7  = leerPotenciometro2(POT7_CHANNEL);
int16_t pot8  = leerPotenciometro2(POT8_CHANNEL);
int16_t pot9  = leerPotenciometro2(POT9_CHANNEL);
int16_t pot10 = leerPotenciometro2(POT10_CHANNEL);

// ===================================================
// SERIAL
// ===================================================

int16_t valores[17] = {flex1, flex2, flex3, flex4, flex5, flex6, flex7,
                       pot1, pot2, pot3, pot4, pot5, pot6, pot7, pot8, pot9, pot10};

// Redibujar 10 veces por segundo (mas rapido no se llega a leer)
static uint32_t ultimoDibujo = 0;
if (millis() - ultimoDibujo >= 100)
{
ultimoDibujo = millis();
dibujarTabla(valores);
}

delay(20);
}