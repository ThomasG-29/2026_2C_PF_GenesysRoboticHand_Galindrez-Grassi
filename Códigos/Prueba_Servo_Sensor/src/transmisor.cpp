// =====================================================
// TRANSMISOR - ESP32-C3 MINI (guante, COM5)
// Lee el flex 1 (multiplexor 2, canal 0 -> A1 del ADS1115),
// calcula su resistencia y la envia por ESP-NOW al receptor.
// =====================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "paquete.h"

// MAC del receptor (ESP32, COM3)
uint8_t macReceptor[] = {0x20, 0x9B, 0xA9, 0x69, 0x0C, 0x00};

// El C3 mini falla con la potencia por defecto (19.5 dBm):
// con 8.5 dBm los envios llegan al 100%.
#define POTENCIA_TX WIFI_POWER_8_5dBm

#define PERIODO_ENVIO_MS 40   // 25 paquetes por segundo

// =====================================================
// HARDWARE (igual que Conexionado_guante-v3)
// =====================================================
#define SDA_PIN 8
#define SCL_PIN 9

#define MUX_S0 10
#define MUX_S1 2
#define MUX_S2 3
#define MUX_S3 4

#define ADS_CANAL_MUX2   1   // SIG del multiplexor 2 -> A1 del ADS1115
#define FLEX1_CANAL_MUX2 0   // flex 1 en el canal 0 del multiplexor 2

Adafruit_ADS1115 ads;

// =====================================================
// DIVISOR DE TENSION DEL FLEX
// El flex va entre VCC y la salida, la resistencia fija entre la
// salida y GND: al doblarlo sube su resistencia y BAJA la lectura.
// =====================================================
#define VCC_DIVISOR 3.3f       // alimentacion del divisor (V)
#define R_FIJA      10000.0f   // resistencia fija del divisor (ohms)

#define MUESTRAS_MEDIANA 5     // lecturas por envio (descarta picos)

// =====================================================

volatile uint32_t ackOk = 0, ackFallidos = 0;

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void onEnviado(const wifi_tx_info_t *info, esp_now_send_status_t estado) {
#else
void onEnviado(const uint8_t *mac, esp_now_send_status_t estado) {
#endif
  if (estado == ESP_NOW_SEND_SUCCESS) ackOk++;
  else ackFallidos++;
}

void selectMuxChannel(uint8_t canal) {
  digitalWrite(MUX_S0, canal & 0x01);
  digitalWrite(MUX_S1, canal & 0x02);
  digitalWrite(MUX_S2, canal & 0x04);
  digitalWrite(MUX_S3, canal & 0x08);
  delayMicroseconds(100);
}

// Mediana de varias lecturas: elimina picos sueltos sin agregar retardo
int16_t leerFlex1() {
  int16_t m[MUESTRAS_MEDIANA];
  for (int i = 0; i < MUESTRAS_MEDIANA; i++) {
    int16_t v = ads.readADC_SingleEnded(ADS_CANAL_MUX2);
    int j = i;
    while (j > 0 && m[j - 1] > v) { m[j] = m[j - 1]; j--; }  // insercion ordenada
    m[j] = v;
  }
  return m[MUESTRAS_MEDIANA / 2];
}

uint32_t calcularResistencia(int16_t adc) {
  float v = ads.computeVolts(adc);
  if (v <= 0.001f) return 9999999;   // sin tension: circuito abierto
  float r = R_FIJA * (VCC_DIVISOR - v) / v;
  return r < 0 ? 0 : (uint32_t)r;
}

void setup() {
  Serial.begin(115200);
  // USB nativo: sin monitor abierto, imprimir espera como maximo 1 ms y
  // descarta el texto (con el valor por defecto la salida se traba tras un
  // rato; con 0 la libreria se queda esperando para siempre)
  Serial.setTxTimeoutMs(1);
  delay(1500);

  Serial.println("\n=== TRANSMISOR FLEX 1 -> SERVO (ESP32-C3) ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ads.begin(0x48)) {
    Serial.println("ERROR: ADS1115 no encontrado.");
    while (true) delay(1000);
  }
  ads.setGain(GAIN_ONE);                     // +-4.096 V
  ads.setDataRate(RATE_ADS1115_860SPS);      // lectura rapida (~1.2 ms)

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  selectMuxChannel(FLEX1_CANAL_MUX2);       // queda fijo: solo se lee el flex 1

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);
  WiFi.setTxPower(POTENCIA_TX);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onEnviado);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, macReceptor, 6);
  peer.channel = CANAL_ESPNOW;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERROR: esp_now_add_peer");
    while (true) delay(1000);
  }

  Serial.printf("Mi MAC: %s  Canal: %d  Envio cada %d ms\n",
                WiFi.macAddress().c_str(), CANAL_ESPNOW, PERIODO_ENVIO_MS);
  Serial.printf("Historial: 1 de cada %d paquetes enviados\n\n", HISTORIAL_CADA_N);
  Serial.println("ENVIADO     tiempo     seq     ADC   tension   resistencia   ACK ok");
}

void loop() {
  static uint32_t proximoEnvio = millis();
  static PaqueteFlex p = {TIPO_FLEX, 0, 0, 0, 0};

  if ((int32_t)(millis() - proximoEnvio) < 0) return;
  proximoEnvio += PERIODO_ENVIO_MS;
  // Si el loop se atraso, se resincroniza en vez de mandar una rafaga de paquetes
  if ((int32_t)(millis() - proximoEnvio) >= 0) proximoEnvio = millis() + PERIODO_ENVIO_MS;

  p.seq++;
  p.t_ms = millis();
  p.adc = leerFlex1();
  p.resistencia = calcularResistencia(p.adc);
  esp_now_send(macReceptor, (uint8_t *)&p, sizeof(p));

  if (p.seq % HISTORIAL_CADA_N == 0) {
    uint32_t ok = ackOk, fail = ackFallidos;
    Serial.printf("ENVIADO  %8.2f s  #%6lu  %6d  %6.3f V  %9lu ohm   %5.1f%%\n",
                  p.t_ms / 1000.0, (unsigned long)p.seq, p.adc, ads.computeVolts(p.adc),
                  (unsigned long)p.resistencia, ok + fail ? 100.0 * ok / (ok + fail) : 0.0);
  }
}
