// =====================================================
// RECEPTOR - ESP32 (mano, COM3)
// Recibe por ESP-NOW la resistencia del flex 1 y mueve el servo
// del pin 25 segun la calibracion (estirado -> doblado).
//
// Comandos por el monitor serie:
//   p : tomar la resistencia actual como ESTIRADO (plano)
//   d : tomar la resistencia actual como DOBLADO
//   r : reiniciar contadores de paquetes
// =====================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include "paquete.h"

// =====================================================
// SERVO
// =====================================================
#define SERVO_PIN      25
#define SERVO_MIN_US   500    // pulso para 0 grados
#define SERVO_MAX_US   2400   // pulso para 180 grados
// Sin llegar a 0/180: en los extremos el servo choca con su tope y vibra
#define ANGULO_PLANO   10     // angulo con el dedo estirado
#define ANGULO_DOBLADO 170    // angulo con el dedo doblado

// Limitador de velocidad (igual que en el guante v2): grados por paquete
#define PASO_MAXIMO 6.0f

// 1: al arrancar, el servo hace un barrido completo (prueba de conexion)
#define BARRIDO_AL_INICIO 1

// Suavizado de la resistencia: mas chico = mas suave pero mas lento
#define ALPHA 0.4f

// =====================================================
// CALIBRACION (ohms) - medida con el flex 1 del guante
// =====================================================
uint32_t rPlano   = 0;   // se mide al arrancar (ver CALIBRAR_PLANO_AL_INICIO)
uint32_t rDoblado = 0;   // se fija en setup() con R_DOBLADO_INICIAL

// Medido con el flex 1 (2026-10-12): estirado 2330-2860 ohm (despues de
// doblarlo vuelve mas bajo y sube de a poco), doblado al maximo 3950-4310 ohm
#define R_PLANO_INICIAL   2900    // por si no se calibra al arrancar
#define R_DOBLADO_INICIAL 4150

// 1: los primeros paquetes (con el dedo ESTIRADO) definen rPlano
#define CALIBRAR_PLANO_AL_INICIO 1
#define PAQUETES_CALIBRACION     25   // ~1 s

#define TIMEOUT_SENAL_MS 500   // sin paquetes durante este tiempo -> SIN SENAL

// =====================================================

Servo servo;
QueueHandle_t cola;

typedef struct {
  PaqueteFlex datos;
  uint32_t t_rx_ms;
} Recepcion;

uint32_t recibidos = 0, perdidos = 0, ultimoSeq = 0, tUltimoRx = 0;
uint32_t ultimaR = 0;
float rFiltrada = -1;
float anguloServo = -1;
int anguloObjetivo = 0;
bool sinSenalAvisado = false;

// Calibracion al inicio
uint32_t calCant = 0;
uint64_t calSuma = 0;
bool calibrando = CALIBRAR_PLANO_AL_INICIO;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecibido(const esp_now_recv_info_t *info, const uint8_t *datos, int largo) {
#else
void onRecibido(const uint8_t *mac, const uint8_t *datos, int largo) {
#endif
  if (largo != sizeof(PaqueteFlex) || datos[0] != TIPO_FLEX) return;
  Recepcion r;
  memcpy(&r.datos, datos, sizeof(PaqueteFlex));
  r.t_rx_ms = millis();
  xQueueSend(cola, &r, 0);
}

void imprimirCalibracion() {
  Serial.printf(">>> Calibracion: estirado = %lu ohm -> %d grados | doblado = %lu ohm -> %d grados\n",
                (unsigned long)rPlano, ANGULO_PLANO, (unsigned long)rDoblado, ANGULO_DOBLADO);
}

void imprimirEncabezado() {
  Serial.println("RECIBIDO    tiempo     seq     ADC   resistencia   angulo   perdidos");
}

// Resistencia -> angulo, segun la calibracion (funciona en cualquier sentido)
int calcularAngulo(float r) {
  if (rPlano == rDoblado) return ANGULO_PLANO;
  float t = (r - (float)rPlano) / ((float)rDoblado - (float)rPlano);
  t = constrain(t, 0.0f, 1.0f);
  return (int)lroundf(ANGULO_PLANO + t * (ANGULO_DOBLADO - ANGULO_PLANO));
}

void moverServoSuave(int objetivo) {
  if (anguloServo < 0) anguloServo = objetivo;
  float dif = objetivo - anguloServo;
  if (fabsf(dif) < 1.0f) return;   // zona muerta: el ruido no hace temblar al servo
  if (fabsf(dif) <= PASO_MAXIMO) anguloServo = objetivo;
  else anguloServo += dif > 0 ? PASO_MAXIMO : -PASO_MAXIMO;
  servo.write((int)lroundf(anguloServo));
}

void procesar(const Recepcion &r) {
  const PaqueteFlex &p = r.datos;

  // El transmisor se reinicio: contadores a cero
  if (ultimoSeq != 0 && p.seq < ultimoSeq) {
    Serial.println(">>> Transmisor reiniciado: contadores a cero");
    recibidos = perdidos = ultimoSeq = 0;
  }
  if (p.seq == ultimoSeq) return;                       // duplicado
  if (ultimoSeq != 0 && p.seq > ultimoSeq + 1) perdidos += p.seq - ultimoSeq - 1;
  ultimoSeq = p.seq;
  recibidos++;
  tUltimoRx = r.t_rx_ms;
  ultimaR = p.resistencia;

  if (sinSenalAvisado) {
    Serial.println(">>> Senal recuperada");
    sinSenalAvisado = false;
  }

  // Calibracion del estirado con los primeros paquetes
  if (calibrando) {
    calSuma += p.resistencia;
    if (++calCant >= PAQUETES_CALIBRACION) {
      rPlano = calSuma / calCant;
      calibrando = false;
      imprimirCalibracion();
      imprimirEncabezado();
    }
    return;
  }

  rFiltrada = rFiltrada < 0 ? p.resistencia : rFiltrada + ALPHA * (p.resistencia - rFiltrada);
  anguloObjetivo = calcularAngulo(rFiltrada);
  moverServoSuave(anguloObjetivo);

  if (p.seq % HISTORIAL_CADA_N == 0) {
    Serial.printf("RECIBIDO %8.2f s  #%6lu  %6d  %9lu ohm   %4d gr   %6lu\n",
                  r.t_rx_ms / 1000.0, (unsigned long)p.seq, p.adc,
                  (unsigned long)p.resistencia, (int)lroundf(anguloServo),
                  (unsigned long)perdidos);
  }
}

void leerComandos() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'p' && ultimaR) { rPlano = ultimaR; imprimirCalibracion(); }
    else if (c == 'd' && ultimaR) { rDoblado = ultimaR; imprimirCalibracion(); }
    else if (c == 'r') {
      recibidos = perdidos = 0;
      Serial.println(">>> Contadores reiniciados");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  rPlano = R_PLANO_INICIAL;
  rDoblado = R_DOBLADO_INICIAL;

  // Motivo del ultimo reinicio (BROWNOUT = cayo la tension de alimentacion)
  const char *motivos[] = {"desconocido", "encendido", "externo", "software", "panico",
                           "watchdog int", "watchdog tarea", "watchdog", "deep sleep",
                           "BROWNOUT", "SDIO"};
  esp_reset_reason_t motivo = esp_reset_reason();
  Serial.printf("\nMotivo del reinicio: %s\n", motivo <= ESP_RST_SDIO ? motivos[motivo] : "?");

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

#if BARRIDO_AL_INICIO
  // Prueba del servo sin depender del flex: barrido lento ida y vuelta
  Serial.println("Prueba del servo: barrido 10 -> 170 -> 10 grados...");
  for (int a = ANGULO_PLANO; a <= ANGULO_DOBLADO; a += 2) { servo.write(a); delay(20); }
  delay(300);
  for (int a = ANGULO_DOBLADO; a >= ANGULO_PLANO; a -= 2) { servo.write(a); delay(20); }
  delay(300);
#endif

  servo.write(ANGULO_PLANO);
  anguloServo = ANGULO_PLANO;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);

  Serial.println("\n=== RECEPTOR FLEX 1 -> SERVO (ESP32) ===");
  Serial.printf("Mi MAC: %s  Canal: %d  Servo en pin %d\n",
                WiFi.macAddress().c_str(), CANAL_ESPNOW, SERVO_PIN);
  Serial.println("Comandos: p = estirado actual | d = doblado actual | r = reiniciar contadores");

  cola = xQueueCreate(20, sizeof(Recepcion));
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecibido);

  if (calibrando)
    Serial.println("Calibrando: mantener el flex ESTIRADO hasta que aparezca la calibracion...");
  else {
    imprimirCalibracion();
    imprimirEncabezado();
  }
}

void loop() {
  Recepcion r;
  if (xQueueReceive(cola, &r, pdMS_TO_TICKS(5)) == pdTRUE) {
    procesar(r);
    while (xQueueReceive(cola, &r, 0) == pdTRUE) procesar(r);
  }

  leerComandos();

  // Sin paquetes: el servo se queda en la ultima posicion
  if (tUltimoRx && !sinSenalAvisado && millis() - tUltimoRx > TIMEOUT_SENAL_MS) {
    Serial.printf(">>> SIN SENAL del transmisor (servo quieto en %d grados)\n", (int)lroundf(anguloServo));
    sinSenalAvisado = true;
  }
}
