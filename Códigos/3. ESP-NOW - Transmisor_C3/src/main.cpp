// =====================================================
// TRANSMISOR ESP-NOW - ESP32-C3 MINI (COM5)
// Envia paquetes al receptor (ESP32) y mide:
//  - ACK de cada envio (confirmacion a nivel radio)
//  - Ecos devueltos por el receptor (entrega extremo a extremo)
//  - Latencia ida y vuelta (RTT), latencia de ida estimada y jitter
//  - Potencia de senal (RSSI) en ambos sentidos
// =====================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// MAC del receptor (ESP32, COM3)
uint8_t macReceptor[] = {0x20, 0x9B, 0xA9, 0x69, 0x0C, 0x00};

// Debe coincidir con el canal del receptor
#define CANAL_ESPNOW 1

// El C3 mini falla con la potencia por defecto (19.5 dBm):
// con 8.5 dBm los envios llegan al 100%.
#define POTENCIA_TX WIFI_POWER_8_5dBm

#define PERIODO_ENVIO_MS   50    // 20 paquetes por segundo
#define PERIODO_REPORTE_MS 1000

// 0: reporte de texto | 1: formato para el Serial Plotter de Arduino
#define MODO_PLOTTER 0

// =====================================================
// PAQUETES: deben ser identicos en el receptor
// =====================================================
#define TIPO_DATOS 1
#define TIPO_ECO   2
#define CANT_FLEX  7

typedef struct __attribute__((packed)) {
  uint8_t  tipo;
  uint32_t seq;          // numero de secuencia
  uint32_t t_envio_us;   // micros() del transmisor al enviar
  int16_t  flex[CANT_FLEX];  // datos de prueba (luego seran los sensores flex)
} PaqueteDatos;

typedef struct __attribute__((packed)) {
  uint8_t  tipo;
  uint32_t seq;          // copia del paquete recibido
  uint32_t t_envio_us;   // copia del paquete recibido
  uint32_t rx_recibidos; // estadisticas del receptor
  uint32_t rx_perdidos;
  uint32_t rx_duplicados;
  int8_t   rssi_rx;      // RSSI con que el receptor escucho el paquete
} PaqueteEco;

// =====================================================
// ESTADISTICAS
// =====================================================
typedef struct {
  uint32_t enviados, ackOk, ackFallidos, ecos;
  uint32_t rttMin, rttMax;          // us
  double   rttSuma, rttSumaCuad;    // us, us^2
  uint64_t tAckSuma;                // us
  uint32_t tAckCant;
  int32_t  rssiEcoSuma, rssiRxSuma;
} Ventana;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Ventana ventana;
uint32_t totEnviados = 0, totAckOk = 0, totAckFallidos = 0, totEcos = 0;
uint32_t rxRecibidos = 0, rxPerdidos = 0, rxDuplicados = 0;  // ultimo dato informado por el receptor

volatile uint32_t tUltimoEnvio_us = 0;
volatile int8_t rssiUltimo = 0;
volatile bool pedidoReinicio = false;
PaqueteDatos paquete;

void reiniciarVentana(Ventana &v) {
  memset(&v, 0, sizeof(v));
  v.rttMin = UINT32_MAX;
}

void reiniciarEstadisticas() {
  portENTER_CRITICAL(&mux);
  totEnviados = totAckOk = totAckFallidos = totEcos = 0;
  rxRecibidos = rxPerdidos = rxDuplicados = 0;
  reiniciarVentana(ventana);
  pedidoReinicio = false;
  portEXIT_CRITICAL(&mux);
}

// Captura el RSSI de las tramas ESP-NOW que llegan del receptor
void onPromiscuo(void *buf, wifi_promiscuous_pkt_type_t tipo) {
  if (tipo != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *trama = pkt->payload;
  // 0xD0 = trama de accion (ESP-NOW); direccion origen en el byte 10
  if (trama[0] == 0xD0 && memcmp(trama + 10, macReceptor, 6) == 0)
    rssiUltimo = pkt->rx_ctrl.rssi;
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void onEnviado(const wifi_tx_info_t *info, esp_now_send_status_t estado) {
#else
void onEnviado(const uint8_t *mac, esp_now_send_status_t estado) {
#endif
  uint32_t dt = micros() - tUltimoEnvio_us;
  portENTER_CRITICAL(&mux);
  if (estado == ESP_NOW_SEND_SUCCESS) {
    ventana.ackOk++;
    totAckOk++;
    ventana.tAckSuma += dt;
    ventana.tAckCant++;
  } else {
    ventana.ackFallidos++;
    totAckFallidos++;
  }
  portEXIT_CRITICAL(&mux);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecibido(const esp_now_recv_info_t *info, const uint8_t *datos, int largo) {
#else
void onRecibido(const uint8_t *mac, const uint8_t *datos, int largo) {
#endif
  uint32_t ahora = micros();
  if (largo != sizeof(PaqueteEco) || datos[0] != TIPO_ECO) return;
  PaqueteEco eco;
  memcpy(&eco, datos, sizeof(eco));
  uint32_t rtt = ahora - eco.t_envio_us;

  portENTER_CRITICAL(&mux);
  ventana.ecos++;
  totEcos++;
  if (rtt < ventana.rttMin) ventana.rttMin = rtt;
  if (rtt > ventana.rttMax) ventana.rttMax = rtt;
  ventana.rttSuma += rtt;
  ventana.rttSumaCuad += (double)rtt * rtt;
  ventana.rssiEcoSuma += rssiUltimo;
  ventana.rssiRxSuma += eco.rssi_rx;
  // Si el contador del receptor bajo, el receptor se reinicio
  if (eco.rx_recibidos < rxRecibidos) pedidoReinicio = true;
  rxRecibidos = eco.rx_recibidos;
  rxPerdidos = eco.rx_perdidos;
  rxDuplicados = eco.rx_duplicados;
  portEXIT_CRITICAL(&mux);
}

const char *calidad(int rssi) {
  if (rssi >= -50) return "Excelente";
  if (rssi >= -65) return "Buena";
  if (rssi >= -75) return "Regular";
  return "Debil";
}

float pct(uint32_t parte, uint32_t total) {
  return total ? 100.0f * parte / total : 0.0f;
}

void reportar() {
  Ventana v;
  uint32_t tEnv, tOk, tFail, tEcos, rRec, rPerd, rDup;
  portENTER_CRITICAL(&mux);
  v = ventana;
  reiniciarVentana(ventana);
  tEnv = totEnviados; tOk = totAckOk; tFail = totAckFallidos; tEcos = totEcos;
  rRec = rxRecibidos; rPerd = rxPerdidos; rDup = rxDuplicados;
  portEXIT_CRITICAL(&mux);

  float rttProm = v.ecos ? v.rttSuma / v.ecos / 1000.0 : 0;
  float jitter = 0;
  if (v.ecos > 1) {
    double media = v.rttSuma / v.ecos;
    double var = v.rttSumaCuad / v.ecos - media * media;
    jitter = var > 0 ? sqrt(var) / 1000.0 : 0;
  }
  float tAckProm = v.tAckCant ? (float)v.tAckSuma / v.tAckCant / 1000.0 : 0;
  int rssiEco = v.ecos ? v.rssiEcoSuma / (int)v.ecos : 0;
  int rssiRx = v.ecos ? v.rssiRxSuma / (int)v.ecos : 0;
  // Perdidos en la ida los detecta el receptor; el resto se perdio en la vuelta
  int32_t perdidosVuelta = (int32_t)tEnv - (int32_t)tEcos - (int32_t)rPerd;
  if (perdidosVuelta < 0) perdidosVuelta = 0;

#if MODO_PLOTTER
  Serial.printf("RTT_ms:%.2f,Jitter_ms:%.2f,ACK_pct:%.1f,Entrega_pct:%.1f,RSSI_ida:%d,RSSI_vuelta:%d\n",
                rttProm, jitter, pct(v.ackOk, v.enviados), pct(v.ecos, v.enviados),
                rssiRx, rssiEco);
#else
  Serial.printf("\n========== TRANSMISOR ESP32-C3 | t = %lu s ==========\n", millis() / 1000);
  if (v.ecos == 0) Serial.println("ESTADO         : SIN RESPUESTA DEL RECEPTOR");
  else             Serial.println("ESTADO         : CONECTADO");
  Serial.println("--- Ultimo segundo ---");
  Serial.printf("Enviados       : %lu  (%.1f paq/s)\n", v.enviados, v.enviados * 1000.0 / PERIODO_REPORTE_MS);
  Serial.printf("ACK OK         : %lu  (%.1f%%)   ACK fallidos: %lu\n", v.ackOk, pct(v.ackOk, v.enviados), v.ackFallidos);
  Serial.printf("Ecos recibidos : %lu  (%.1f%% entrega ida y vuelta)\n", v.ecos, pct(v.ecos, v.enviados));
  if (v.ecos) {
    Serial.printf("Latencia RTT   : prom %.2f ms | min %.2f | max %.2f | jitter %.2f ms\n",
                  rttProm, v.rttMin / 1000.0, v.rttMax / 1000.0, jitter);
    Serial.printf("Latencia ida   : ~%.2f ms (RTT / 2)\n", rttProm / 2);
    Serial.printf("Tiempo a ACK   : %.2f ms (envio -> confirmacion de radio)\n", tAckProm);
    Serial.printf("RSSI ida       : %d dBm [%s]  (medido en el receptor)\n", rssiRx, calidad(rssiRx));
    Serial.printf("RSSI vuelta    : %d dBm [%s]  (medido en el transmisor)\n", rssiEco, calidad(rssiEco));
  }
  Serial.println("--- Acumulado ---");
  Serial.printf("Enviados       : %lu | ACK OK %lu (%.2f%%) | ACK fallidos %lu\n", tEnv, tOk, pct(tOk, tEnv), tFail);
  Serial.printf("Ecos           : %lu (%.2f%%)\n", tEcos, pct(tEcos, tEnv));
  Serial.printf("Receptor       : recibidos %lu | perdidos en ida %lu | duplicados %lu\n", rRec, rPerd, rDup);
  Serial.printf("Perdidos vuelta: %ld\n", (long)perdidosVuelta);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);
  WiFi.setTxPower(POTENCIA_TX);

  Serial.println("\n=== TRANSMISOR (ESP32-C3) ===");
  Serial.printf("Mi MAC: %s  Canal: %d  Periodo: %d ms\n",
                WiFi.macAddress().c_str(), CANAL_ESPNOW, PERIODO_ENVIO_MS);
  Serial.println("Enviar 'r' para reiniciar estadisticas");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onEnviado);
  esp_now_register_recv_cb(onRecibido);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, macReceptor, 6);
  peer.channel = CANAL_ESPNOW;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERROR: esp_now_add_peer");
    while (true) delay(1000);
  }

  // Modo promiscuo solo para leer el RSSI de los ecos
  wifi_promiscuous_filter_t filtro = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
  esp_wifi_set_promiscuous_filter(&filtro);
  esp_wifi_set_promiscuous_rx_cb(onPromiscuo);
  esp_wifi_set_promiscuous(true);

  reiniciarVentana(ventana);
  paquete.tipo = TIPO_DATOS;
}

void loop() {
  // El reporte se desfasa medio periodo de los envios para que el ACK y
  // el eco de cada paquete se cuenten en el mismo segundo en que se envio
  static uint32_t proximoEnvio = millis();
  static uint32_t proximoReporte = proximoEnvio + PERIODO_REPORTE_MS + PERIODO_ENVIO_MS / 2;

  // Enviar 'r' desde el monitor serie reinicia las estadisticas
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') pedidoReinicio = true;
  }
  if (pedidoReinicio) {
    reiniciarEstadisticas();
    Serial.println("\n>>> Estadisticas reiniciadas");
  }

  if ((int32_t)(millis() - proximoReporte) >= 0) {
    proximoReporte += PERIODO_REPORTE_MS;
    reportar();
  }

  if ((int32_t)(millis() - proximoEnvio) >= 0) {
    proximoEnvio += PERIODO_ENVIO_MS;
    paquete.seq++;
    for (int i = 0; i < CANT_FLEX; i++)
      paquete.flex[i] = (paquete.seq * 10 + i * 100) % 1024;  // valores simulados
    portENTER_CRITICAL(&mux);
    ventana.enviados++;
    totEnviados++;
    portEXIT_CRITICAL(&mux);
    tUltimoEnvio_us = micros();
    paquete.t_envio_us = tUltimoEnvio_us;
    esp_now_send(macReceptor, (uint8_t *)&paquete, sizeof(paquete));
  }
}
