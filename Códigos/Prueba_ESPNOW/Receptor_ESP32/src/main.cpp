// =====================================================
// RECEPTOR ESP-NOW - ESP32 (COM3)
// Recibe los paquetes del transmisor (ESP32-C3), devuelve un eco
// para que el transmisor mida la latencia, y mide:
//  - Paquetes recibidos, perdidos, duplicados y desordenados
//  - Intervalo entre paquetes (regularidad del envio)
//  - Potencia de senal (RSSI)
// =====================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Debe coincidir con el canal del transmisor
#define CANAL_ESPNOW 1

#define PERIODO_REPORTE_MS 1000
#define TIMEOUT_SENAL_MS   1000   // sin paquetes durante este tiempo -> SIN SENAL

// 0: reporte de texto | 1: formato para el Serial Plotter de Arduino
#define MODO_PLOTTER 0

// =====================================================
// PAQUETES: deben ser identicos en el transmisor
// =====================================================
#define TIPO_DATOS 1
#define TIPO_ECO   2
#define CANT_FLEX  7

typedef struct __attribute__((packed)) {
  uint8_t  tipo;
  uint32_t seq;
  uint32_t t_envio_us;
  int16_t  flex[CANT_FLEX];
} PaqueteDatos;

typedef struct __attribute__((packed)) {
  uint8_t  tipo;
  uint32_t seq;
  uint32_t t_envio_us;
  uint32_t rx_recibidos;
  uint32_t rx_perdidos;
  uint32_t rx_duplicados;
  int8_t   rssi_rx;
} PaqueteEco;

// Lo que el callback le pasa al loop
typedef struct {
  PaqueteDatos datos;
  uint8_t mac[6];
  int8_t rssi;
  uint32_t t_rx_us;
} Recepcion;

QueueHandle_t cola;
volatile int8_t rssiUltimo = 0;
uint8_t miMac[6];
uint8_t macTransmisor[6];
bool hayTransmisor = false;

// =====================================================
// ESTADISTICAS
// =====================================================
typedef struct {
  uint32_t recibidos, perdidos, duplicados, desordenados;
  uint32_t intMin, intMax;   // intervalo entre paquetes, us
  uint64_t intSuma;
  uint32_t intCant;
  int32_t  rssiSuma;
  int8_t   rssiMin, rssiMax;
} Ventana;

Ventana ventana;
uint32_t totRecibidos = 0, totPerdidos = 0, totDuplicados = 0, totDesordenados = 0;
uint32_t ultimoSeq = 0, tUltimoRx_us = 0, tUltimoRx_ms = 0;
uint32_t colaLlena = 0;
PaqueteDatos ultimoPaquete;

void reiniciarVentana(Ventana &v) {
  memset(&v, 0, sizeof(v));
  v.intMin = UINT32_MAX;
  v.rssiMin = 127;
  v.rssiMax = -128;
}

// Captura el RSSI de las tramas ESP-NOW (tramas de accion) dirigidas a este ESP32
void onPromiscuo(void *buf, wifi_promiscuous_pkt_type_t tipo) {
  if (tipo != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *trama = pkt->payload;
  // 0xD0 = trama de accion (ESP-NOW); direccion destino en el byte 4
  if (trama[0] == 0xD0 && memcmp(trama + 4, miMac, 6) == 0)
    rssiUltimo = pkt->rx_ctrl.rssi;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecibido(const esp_now_recv_info_t *info, const uint8_t *datos, int largo) {
  const uint8_t *mac = info->src_addr;
#else
void onRecibido(const uint8_t *mac, const uint8_t *datos, int largo) {
#endif
  if (largo != sizeof(PaqueteDatos) || datos[0] != TIPO_DATOS) return;
  Recepcion r;
  memcpy(&r.datos, datos, sizeof(PaqueteDatos));
  memcpy(r.mac, mac, 6);
  r.rssi = rssiUltimo;
  r.t_rx_us = micros();
  if (xQueueSend(cola, &r, 0) != pdTRUE) colaLlena++;
}

void registrarTransmisor(const uint8_t *mac) {
  memcpy(macTransmisor, mac, 6);
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = CANAL_ESPNOW;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }
  hayTransmisor = true;
  Serial.printf("Transmisor registrado: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void procesar(const Recepcion &r) {
  if (!hayTransmisor) registrarTransmisor(r.mac);

  // Eco inmediato al transmisor para que mida la latencia
  PaqueteEco eco;
  eco.tipo = TIPO_ECO;
  eco.seq = r.datos.seq;
  eco.t_envio_us = r.datos.t_envio_us;
  eco.rx_recibidos = totRecibidos + 1;
  eco.rx_perdidos = totPerdidos;
  eco.rx_duplicados = totDuplicados;
  eco.rssi_rx = r.rssi;
  esp_now_send(macTransmisor, (uint8_t *)&eco, sizeof(eco));

  uint32_t seq = r.datos.seq;

  // El transmisor se reinicio: se reinician las estadisticas
  if (ultimoSeq != 0 && seq + 100 < ultimoSeq) {
    Serial.println(">>> Transmisor reiniciado: estadisticas a cero");
    totRecibidos = totPerdidos = totDuplicados = totDesordenados = 0;
    ultimoSeq = 0;
    reiniciarVentana(ventana);
  }

  if (seq == ultimoSeq) {            // retransmision del mismo paquete
    ventana.duplicados++;
    totDuplicados++;
    return;
  }
  if (ultimoSeq != 0 && seq < ultimoSeq) {   // llego tarde: ya se habia contado como perdido
    ventana.desordenados++;
    totDesordenados++;
    if (totPerdidos) totPerdidos--;
    return;
  }

  if (ultimoSeq != 0 && seq > ultimoSeq + 1) {
    uint32_t huecos = seq - ultimoSeq - 1;
    ventana.perdidos += huecos;
    totPerdidos += huecos;
  }

  if (ultimoSeq != 0) {
    uint32_t intervalo = r.t_rx_us - tUltimoRx_us;
    if (intervalo < ventana.intMin) ventana.intMin = intervalo;
    if (intervalo > ventana.intMax) ventana.intMax = intervalo;
    ventana.intSuma += intervalo;
    ventana.intCant++;
  }

  ultimoSeq = seq;
  tUltimoRx_us = r.t_rx_us;
  tUltimoRx_ms = millis();
  ventana.recibidos++;
  totRecibidos++;
  ventana.rssiSuma += r.rssi;
  if (r.rssi < ventana.rssiMin) ventana.rssiMin = r.rssi;
  if (r.rssi > ventana.rssiMax) ventana.rssiMax = r.rssi;
  ultimoPaquete = r.datos;
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
  Ventana v = ventana;
  reiniciarVentana(ventana);

  bool conectado = tUltimoRx_ms != 0 && millis() - tUltimoRx_ms < TIMEOUT_SENAL_MS;
  int rssiProm = v.recibidos ? v.rssiSuma / (int)v.recibidos : 0;
  float intProm = v.intCant ? (float)v.intSuma / v.intCant / 1000.0 : 0;

#if MODO_PLOTTER
  Serial.printf("Recibidos:%lu,Perdidos:%lu,Perdida_pct:%.1f,RSSI:%d,Intervalo_ms:%.2f\n",
                (unsigned long)v.recibidos, (unsigned long)v.perdidos,
                pct(v.perdidos, v.recibidos + v.perdidos), rssiProm, intProm);
#else
  Serial.printf("\n========== RECEPTOR ESP32 | t = %lu s ==========\n", millis() / 1000);
  if (conectado) Serial.println("ESTADO         : CONECTADO");
  else if (tUltimoRx_ms) Serial.printf("ESTADO         : SIN SENAL (ultimo paquete hace %lu ms)\n", millis() - tUltimoRx_ms);
  else Serial.println("ESTADO         : ESPERANDO AL TRANSMISOR...");

  Serial.println("--- Ultimo segundo ---");
  Serial.printf("Recibidos      : %lu  (%.1f paq/s)\n", (unsigned long)v.recibidos,
                v.recibidos * 1000.0 / PERIODO_REPORTE_MS);
  Serial.printf("Perdidos       : %lu  (%.1f%%)   Duplicados: %lu   Desordenados: %lu\n",
                (unsigned long)v.perdidos, pct(v.perdidos, v.recibidos + v.perdidos),
                (unsigned long)v.duplicados, (unsigned long)v.desordenados);
  if (v.recibidos) {
    if (v.intCant)
      Serial.printf("Intervalo      : prom %.2f ms | min %.2f | max %.2f ms\n",
                    intProm, v.intMin / 1000.0, v.intMax / 1000.0);
    Serial.printf("RSSI           : prom %d dBm | min %d | max %d  [%s]\n",
                  rssiProm, v.rssiMin, v.rssiMax, calidad(rssiProm));
    Serial.printf("Ultimo paquete : #%lu  flex =", (unsigned long)ultimoPaquete.seq);
    for (int i = 0; i < CANT_FLEX; i++) Serial.printf(" %d", ultimoPaquete.flex[i]);
    Serial.println();
  }
  Serial.println("--- Acumulado ---");
  Serial.printf("Recibidos      : %lu | perdidos %lu (%.2f%%) | duplicados %lu | desordenados %lu\n",
                (unsigned long)totRecibidos, (unsigned long)totPerdidos,
                pct(totPerdidos, totRecibidos + totPerdidos),
                (unsigned long)totDuplicados, (unsigned long)totDesordenados);
  if (colaLlena) Serial.printf("AVISO: %lu paquetes descartados por cola llena\n", (unsigned long)colaLlena);
#endif
}

void setup() {
  Serial.setTxBufferSize(1024);  // imprimir sin frenar el procesamiento de paquetes
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);
  esp_wifi_get_mac(WIFI_IF_STA, miMac);

  Serial.println("\n=== RECEPTOR (ESP32) ===");
  Serial.printf("Mi MAC: %s  Canal: %d\n", WiFi.macAddress().c_str(), CANAL_ESPNOW);

  cola = xQueueCreate(20, sizeof(Recepcion));
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecibido);

  // Modo promiscuo solo para leer el RSSI de los paquetes
  wifi_promiscuous_filter_t filtro = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
  esp_wifi_set_promiscuous_filter(&filtro);
  esp_wifi_set_promiscuous_rx_cb(onPromiscuo);
  esp_wifi_set_promiscuous(true);

  reiniciarVentana(ventana);
  Serial.println("Esperando paquetes... (enviar 'r' para reiniciar estadisticas)");
}

void loop() {
  static uint32_t ultimoReporte = 0;
  Recepcion r;

  // Espera hasta 5 ms por un paquete para no bloquear el reporte
  if (xQueueReceive(cola, &r, pdMS_TO_TICKS(5)) == pdTRUE) {
    procesar(r);
    while (xQueueReceive(cola, &r, 0) == pdTRUE) procesar(r);
  }

  // Enviar 'r' desde el monitor serie reinicia las estadisticas
  // (el transmisor lo detecta y reinicia las suyas tambien)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      totRecibidos = totPerdidos = totDuplicados = totDesordenados = 0;
      colaLlena = 0;
      reiniciarVentana(ventana);
      Serial.println("\n>>> Estadisticas reiniciadas");
    }
  }

  if (millis() - ultimoReporte >= PERIODO_REPORTE_MS) {
    ultimoReporte = millis();
    reportar();
  }
}
