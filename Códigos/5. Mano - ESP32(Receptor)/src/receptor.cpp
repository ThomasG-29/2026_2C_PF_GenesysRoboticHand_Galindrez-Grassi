// =====================================================
// RECEPTOR - ESP32 (mano robot, COM3)
// Recibe por ESP-NOW la resistencia de los 7 flex del guante y mueve
// un servo por flex (PCA9685) segun la calibracion (estirado -> doblado).
//
//   Flex 1 al 5 -> MG90S  S1 a S5   (canales 0 a 4 del PCA9685)
//   Flex 6 y 7  -> DS3225 S11 y S12 (canales 10 y 11 del PCA9685)
//   (segun Conexionados/Circuito_mano_robot)
//
// Flex 7 (costado de la muneca) es BIDIRECCIONAL: muneca derecha = centro
// (servo a 90), doblada hacia un lado sube la resistencia (servo hacia
// anguloDoblado) y doblada hacia el otro baja (servo hacia anguloOtroLado).
// Si el servo gira al reves, intercambiar esos dos angulos (a7 90 10 170).
//
// La calibracion de los flex se guarda en la memoria del ESP32
// (no se pierde al apagar). Al arrancar, el estirado se vuelve a medir
// con los primeros paquetes: arrancar con la mano del guante ESTIRADA
// y la muneca DERECHA (centro).
//
// Comandos por el monitor serie (escribir y Enter):
//   c      : calibracion guiada (mano abierta 3 s, despues doblar todo 6 s;
//            en esos 6 s llevar la muneca a un costado y despues al otro)
//   p / d  : tomar la resistencia actual de TODOS los flex como estirado / doblado
//   p3, d3 : lo mismo pero solo para el flex 3 (1 a 7)
//   o7     : tomar la resistencia actual del flex 7 como "otro lado"
//            (solo flex bidireccionales)
//   s      : mostrar la calibracion
//   t3     : prueba del servo del flex 3 (barrido estirado -> doblado -> estirado;
//            si es bidireccional sigue -> otro lado -> centro)
//   a3 10 170 : angulos del servo 3 (estirado, doblado). Es temporal, para
//               probar: los definitivos se escriben en SERVOS[] mas abajo.
//   a7 90 170 10 : angulos del servo 7 (centro, un lado, otro lado)
//   r      : reiniciar contadores de paquetes
// =====================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include "paquete.h"

// =====================================================
// SERVOS
// =====================================================
// Pulso para 0 y 180 grados de cada modelo (igual que Servomotores_mano_robot)
#define MG90S_MIN_US  500
#define MG90S_MAX_US  2400
#define DS3225_MIN_US 500
#define DS3225_MAX_US 2500

typedef struct {
  const char *nombre;
  uint8_t  canal;           // salida del PCA9685
  uint16_t minUs, maxUs;    // pulso para 0 y 180 grados
  // Sin llegar a 0/180: en los extremos el servo choca con su tope y vibra
  uint8_t  anguloPlano;     // angulo con el dedo estirado (centro si es bidireccional)
  uint8_t  anguloDoblado;   // angulo con el dedo doblado (lado donde SUBE la resistencia)
  uint8_t  anguloOtroLado;  // bidireccional: angulo doblado hacia el lado donde BAJA
                            // la resistencia. SIN_OTRO_LADO = flex comun (un solo sentido)
} ServoDedo;

#define SIN_OTRO_LADO 255

// Un servo por flex, en el mismo orden (SERVOS[0] lo mueve el flex 1)
ServoDedo SERVOS[CANT_FLEX] = {
  {"S1 MG90S",    0, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO},   // Flex 1
  {"S2 MG90S",    1, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO},   // Flex 2
  {"S3 MG90S",    2, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO},   // Flex 3
  {"S4 MG90S",    3, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO},   // Flex 4
  {"S5 MG90S",    4, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO},   // Flex 5
  {"S11 DS3225", 10, DS3225_MIN_US, DS3225_MAX_US, 10, 170, SIN_OTRO_LADO},   // Flex 6
  {"S12 DS3225", 11, DS3225_MIN_US, DS3225_MAX_US, 90, 170, 10},              // Flex 7 (muneca lateral)
};

bool esBidireccional(int i) { return SERVOS[i].anguloOtroLado != SIN_OTRO_LADO; }

// Limitador de velocidad (igual que en la prueba): grados por paquete
#define PASO_MAXIMO 6.0f

// Suavizado de la resistencia: mas chico = mas suave pero mas lento
#define ALPHA 0.4f

#define I2C_SDA 21
#define I2C_SCL 22

// =====================================================
// CALIBRACION DE LOS FLEX (ohms)
// =====================================================
// Valores por si nunca se calibro (medidos con el flex 1 en la prueba).
// Cada flex tiene distinta resistencia estirado (2800 a 5000 ohm), asi que
// sin calibracion el doblado se toma como estirado + RANGO_INICIAL.
#define R_PLANO_INICIAL   2900
#define RANGO_INICIAL     1300

// 1: los primeros paquetes (con la mano ESTIRADA) definen el estirado
#define CALIBRAR_PLANO_AL_INICIO 1
#define PAQUETES_CALIBRACION     25   // ~1 s

// Calibracion guiada (comando c)
#define CAL_ABIERTA_MS 3000
#define CAL_CERRADA_MS 6000
#define RANGO_MINIMO   300     // ohms: si un flex cambia menos, no se calibra
#define MARGEN_DOBLADO 0.05f   // el doblado queda un 5% antes del maximo medido

// Flex bidireccional (muneca): hacia el lado donde BAJA la resistencia
// el flex cambia bastante menos, por eso el rango y el minimo son menores
#define RANGO_INICIAL_OTRO_LADO 500   // ohms por debajo del centro si nunca se calibro
#define RANGO_MINIMO_OTRO_LADO  100   // ohms: si cambia menos, no se calibra ese lado
#define ZONA_CENTRO 0.06f             // 6% del recorrido alrededor del centro = servo quieto en el centro

#define TIMEOUT_SENAL_MS 500   // sin paquetes durante este tiempo -> SIN SENAL

// =====================================================

Adafruit_PWMServoDriver pca(0x40);
Preferences memoria;
QueueHandle_t cola;

typedef struct {
  PaqueteGuante datos;
  uint32_t t_rx_ms;
} Recepcion;

uint32_t rPlano[CANT_FLEX], rDoblado[CANT_FLEX];
uint32_t rOtroLado[CANT_FLEX];   // solo flex bidireccionales: resistencia doblado hacia el otro lado
bool calibracionGuardada = false;
float rFiltrada[CANT_FLEX];
float anguloServo[CANT_FLEX];
uint32_t ultimaR[CANT_FLEX];
bool hayDatos = false;

uint32_t recibidos = 0, perdidos = 0, ultimoSeq = 0, tUltimoRx = 0;
volatile uint32_t largoDistinto = 0;
bool sinSenalAvisado = false;

// Calibracion del estirado al inicio
bool calibrandoInicio = CALIBRAR_PLANO_AL_INICIO;
uint32_t calCant = 0;
uint64_t calSuma[CANT_FLEX];

// Calibracion guiada
enum { CAL_NO, CAL_ABIERTA, CAL_CERRADA } calEstado = CAL_NO;
uint32_t calInicio = 0, calUltimoAviso = 0;
float calMax[CANT_FLEX], calMin[CANT_FLEX];

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecibido(const esp_now_recv_info_t *info, const uint8_t *datos, int largo) {
#else
void onRecibido(const uint8_t *mac, const uint8_t *datos, int largo) {
#endif
  if (largo != sizeof(PaqueteGuante)) { largoDistinto++; return; }
  if (datos[0] != TIPO_GUANTE) return;
  Recepcion r;
  memcpy(&r.datos, datos, sizeof(PaqueteGuante));
  r.t_rx_ms = millis();
  xQueueSend(cola, &r, 0);
}

// =====================================================
// SERVOS (PCA9685 a 50 Hz: 20000 us en 4096 pasos)
// =====================================================
void escribirServo(int i, float angulo) {
  const ServoDedo &s = SERVOS[i];
  angulo = constrain(angulo, 0.0f, 180.0f);
  float us = s.minUs + angulo * (s.maxUs - s.minUs) / 180.0f;
  pca.setPWM(s.canal, 0, (uint16_t)lroundf(us * 4096.0f / 20000.0f));
}

void moverServoSuave(int i, float objetivo) {
  float dif = objetivo - anguloServo[i];
  if (fabsf(dif) < 1.0f) return;   // zona muerta: el ruido no hace temblar al servo
  if (fabsf(dif) <= PASO_MAXIMO) anguloServo[i] = objetivo;
  else anguloServo[i] += dif > 0 ? PASO_MAXIMO : -PASO_MAXIMO;
  escribirServo(i, anguloServo[i]);
}

// Bidireccional: cerca del centro el servo queda quieto en el centro
// (el ruido de la muneca derecha no lo hace cabecear de un lado al otro)
float aplicarZonaCentro(float t) {
  return t <= ZONA_CENTRO ? 0.0f : (t - ZONA_CENTRO) / (1.0f - ZONA_CENTRO);
}

// Resistencia -> angulo, segun la calibracion (funciona en cualquier sentido)
float calcularAngulo(int i, float r) {
  const ServoDedo &s = SERVOS[i];
  if (rPlano[i] == rDoblado[i]) return s.anguloPlano;
  float t = (r - (float)rPlano[i]) / ((float)rDoblado[i] - (float)rPlano[i]);

  if (esBidireccional(i)) {
    if (t < 0) {
      // Doblado hacia el otro lado (la resistencia va al reves que en el doblado)
      if (rOtroLado[i] == rPlano[i]) return s.anguloPlano;
      t = (r - (float)rPlano[i]) / ((float)rOtroLado[i] - (float)rPlano[i]);
      t = aplicarZonaCentro(constrain(t, 0.0f, 1.0f));
      return s.anguloPlano + t * (s.anguloOtroLado - s.anguloPlano);
    }
    t = aplicarZonaCentro(constrain(t, 0.0f, 1.0f));
  }

  t = constrain(t, 0.0f, 1.0f);
  return s.anguloPlano + t * (s.anguloDoblado - s.anguloPlano);
}

// Barrido lento de un servo entre dos angulos
void barrerServo(int i, int desde, int hasta) {
  int paso = hasta > desde ? 2 : -2;
  for (int a = desde; (a - hasta) * paso <= 0; a += paso) {
    escribirServo(i, a);
    delay(25);
  }
  escribirServo(i, hasta);
}

// Prueba de conexion y de recorrido de un servo
void probarServo(int i) {
  const ServoDedo &s = SERVOS[i];
  if (esBidireccional(i))
    Serial.printf(">>> Prueba %s (flex %d, canal %d): %d -> %d -> %d -> %d -> %d grados\n",
                  s.nombre, i + 1, s.canal, s.anguloPlano, s.anguloDoblado, s.anguloPlano,
                  s.anguloOtroLado, s.anguloPlano);
  else
    Serial.printf(">>> Prueba %s (flex %d, canal %d): %d -> %d -> %d grados\n",
                  s.nombre, i + 1, s.canal, s.anguloPlano, s.anguloDoblado, s.anguloPlano);
  barrerServo(i, s.anguloPlano, s.anguloDoblado);
  delay(400);
  barrerServo(i, s.anguloDoblado, s.anguloPlano);
  if (esBidireccional(i)) {
    delay(400);
    barrerServo(i, s.anguloPlano, s.anguloOtroLado);
    delay(400);
    barrerServo(i, s.anguloOtroLado, s.anguloPlano);
  }
  anguloServo[i] = s.anguloPlano;
  escribirServo(i, anguloServo[i]);
  xQueueReset(cola);   // descarta los paquetes que llegaron durante la prueba
  ultimoSeq = 0;
  tUltimoRx = millis();   // la pausa de la prueba no cuenta como SIN SENAL
}

// =====================================================
// CALIBRACION
// =====================================================
void guardarCalibracion() {
  memoria.putBytes("rPlano", rPlano, sizeof(rPlano));
  memoria.putBytes("rDoblado", rDoblado, sizeof(rDoblado));
  memoria.putBytes("rOtroLado", rOtroLado, sizeof(rOtroLado));
  calibracionGuardada = true;
}

// "Otro lado" por defecto: un poco por debajo del centro
uint32_t otroLadoInicial(uint32_t plano) {
  return plano > 2 * RANGO_INICIAL_OTRO_LADO ? plano - RANGO_INICIAL_OTRO_LADO : plano / 2;
}

// El otro lado tiene que quedar por debajo del centro (el doblado queda por encima)
bool otroLadoValido(int i) {
  return rOtroLado[i] + RANGO_MINIMO_OTRO_LADO <= rPlano[i];
}

void cargarCalibracion() {
  for (int i = 0; i < CANT_FLEX; i++) {
    rPlano[i] = R_PLANO_INICIAL;
    rDoblado[i] = R_PLANO_INICIAL + RANGO_INICIAL;
    rOtroLado[i] = otroLadoInicial(R_PLANO_INICIAL);
  }
  bool ok = memoria.isKey("rPlano") && memoria.isKey("rDoblado") &&
            memoria.getBytesLength("rPlano") == sizeof(rPlano) &&
            memoria.getBytesLength("rDoblado") == sizeof(rDoblado);
  calibracionGuardada = ok;
  if (ok) {
    memoria.getBytes("rPlano", rPlano, sizeof(rPlano));
    memoria.getBytes("rDoblado", rDoblado, sizeof(rDoblado));
    // Calibraciones viejas no tienen el otro lado: se usa el valor por defecto
    if (memoria.isKey("rOtroLado") && memoria.getBytesLength("rOtroLado") == sizeof(rOtroLado))
      memoria.getBytes("rOtroLado", rOtroLado, sizeof(rOtroLado));
    for (int i = 0; i < CANT_FLEX; i++)
      if (!otroLadoValido(i)) rOtroLado[i] = otroLadoInicial(rPlano[i]);
  }
  Serial.println(ok ? "Calibracion cargada de la memoria." : "Sin calibracion guardada: valores por defecto.");
}

void imprimirCalibracion() {
  Serial.println(">>> Calibracion:  flex  servo        canal   estirado (ohm -> gr)   doblado (ohm -> gr)   otro lado (ohm -> gr)");
  for (int i = 0; i < CANT_FLEX; i++) {
    const ServoDedo &s = SERVOS[i];
    Serial.printf(">>>                 %d   %-11s   %2d    %6lu -> %3d         %6lu -> %3d",
                  i + 1, s.nombre, s.canal, (unsigned long)rPlano[i], s.anguloPlano,
                  (unsigned long)rDoblado[i], s.anguloDoblado);
    if (esBidireccional(i))
      Serial.printf("          %6lu -> %3d", (unsigned long)rOtroLado[i], s.anguloOtroLado);
    Serial.println();
  }
}

void imprimirEncabezado() {
  Serial.print("RECIBIDO    tiempo     seq |");
  for (int i = 0; i < CANT_FLEX; i++) Serial.printf("    R%d", i + 1);
  Serial.print(" (ohm) |");
  for (int i = 0; i < CANT_FLEX; i++) Serial.printf("  A%d", i + 1);
  Serial.println(" (gr) | perdidos");
}

void terminarCalibracionInicio() {
  for (int i = 0; i < CANT_FLEX; i++) {
    uint32_t medido = calSuma[i] / calCant;
    // Si el flex ya estaba doblado al arrancar, se deja el estirado guardado
    int32_t rango = (int32_t)rDoblado[i] - (int32_t)rPlano[i];
    if (calibracionGuardada && (int32_t)(medido - rPlano[i]) > rango / 2) {
      Serial.printf(">>> Flex %d parece doblado al arrancar (%lu ohm): se deja el estirado guardado\n",
                    i + 1, (unsigned long)medido);
      continue;
    }
    // Bidireccional: tambien puede estar doblado hacia el otro lado
    int32_t rangoOtro = (int32_t)rPlano[i] - (int32_t)rOtroLado[i];
    if (esBidireccional(i) && calibracionGuardada &&
        (int32_t)rPlano[i] - (int32_t)medido > rangoOtro / 2) {
      Serial.printf(">>> Flex %d parece doblado hacia el otro lado al arrancar (%lu ohm): se deja el centro guardado\n",
                    i + 1, (unsigned long)medido);
      continue;
    }
    // El otro lado se mueve junto con el centro (mantiene su rango)
    if (esBidireccional(i) && calibracionGuardada && otroLadoValido(i) && (int32_t)medido > rangoOtro)
      rOtroLado[i] = medido - rangoOtro;
    rPlano[i] = medido;
    // Sin un doblado valido (nunca se calibro): estirado + rango tipico
    if (!calibracionGuardada || rDoblado[i] < rPlano[i] + RANGO_MINIMO)
      rDoblado[i] = rPlano[i] + RANGO_INICIAL;
    if (!calibracionGuardada || !otroLadoValido(i))
      rOtroLado[i] = otroLadoInicial(rPlano[i]);
  }
  calibrandoInicio = false;
  imprimirCalibracion();
  imprimirEncabezado();
}

void iniciarCalibracionGuiada() {
  if (!hayDatos || calibrandoInicio) { Serial.println(">>> Sin datos del guante: no se puede calibrar"); return; }
  calEstado = CAL_ABIERTA;
  calInicio = calUltimoAviso = millis();
  calCant = 0;
  for (int i = 0; i < CANT_FLEX; i++) calSuma[i] = 0;
  Serial.println("\n>>> CALIBRACION 1/2: ABRIR la mano (todos los dedos estirados, muneca DERECHA) y dejarla quieta 3 s...");
}

void avanzarCalibracionGuiada() {
  uint32_t ahora = millis();
  if (calEstado == CAL_ABIERTA && ahora - calInicio >= CAL_ABIERTA_MS) {
    if (calCant == 0) { Serial.println(">>> Sin datos: calibracion cancelada"); calEstado = CAL_NO; return; }
    for (int i = 0; i < CANT_FLEX; i++) calMax[i] = calMin[i] = calSuma[i] / calCant;
    calEstado = CAL_CERRADA;
    calInicio = calUltimoAviso = ahora;
    Serial.println(">>> CALIBRACION 2/2: DOBLAR AL MAXIMO todos los flex (cerrar la mano, etc.) durante 6 s...");
    Serial.println(">>>                  MUNECA: llevarla al maximo hacia un costado y despues hacia el otro");
  } else if (calEstado == CAL_CERRADA && ahora - calInicio >= CAL_CERRADA_MS) {
    calEstado = CAL_NO;
    for (int i = 0; i < CANT_FLEX; i++) {
      uint32_t plano = calSuma[i] / calCant;
      float rango = calMax[i] - plano;
      if (rango < RANGO_MINIMO) {
        Serial.printf(">>> Flex %d casi no cambio (%.0f ohm): se deja la calibracion anterior\n", i + 1, rango);
        continue;
      }
      // Bidireccional: el otro lado es el minimo medido (si el centro se mueve,
      // un otro lado viejo puede quedar mal, por eso se revisa despues)
      if (esBidireccional(i)) {
        float rangoOtro = plano - calMin[i];
        if (rangoOtro >= RANGO_MINIMO_OTRO_LADO)
          rOtroLado[i] = (uint32_t)(calMin[i] + MARGEN_DOBLADO * rangoOtro);
        else
          Serial.printf(">>> Flex %d casi no cambio hacia el otro lado (%.0f ohm): se deja el anterior\n",
                        i + 1, rangoOtro);
      }
      rPlano[i] = plano;
      rDoblado[i] = (uint32_t)(calMax[i] - MARGEN_DOBLADO * rango);
      if (esBidireccional(i) && !otroLadoValido(i)) rOtroLado[i] = otroLadoInicial(rPlano[i]);
    }
    guardarCalibracion();
    Serial.println(">>> Calibracion terminada y guardada.");
    imprimirCalibracion();
    imprimirEncabezado();
  } else if (ahora - calUltimoAviso >= 1000) {
    calUltimoAviso = ahora;
    uint32_t total = calEstado == CAL_ABIERTA ? CAL_ABIERTA_MS : CAL_CERRADA_MS;
    Serial.printf(">>> ... %lu s\n", (unsigned long)((total - (ahora - calInicio) + 999) / 1000));
  }
}

// =====================================================
// RECEPCION
// =====================================================
void procesar(const Recepcion &r) {
  const PaqueteGuante &p = r.datos;

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
  hayDatos = true;

  if (sinSenalAvisado) {
    Serial.println(">>> Senal recuperada");
    sinSenalAvisado = false;
  }

  for (int i = 0; i < CANT_FLEX; i++) {
    ultimaR[i] = p.resistencia[i];
    rFiltrada[i] = rFiltrada[i] < 0 ? p.resistencia[i]
                                    : rFiltrada[i] + ALPHA * (p.resistencia[i] - rFiltrada[i]);
  }

  // Calibracion del estirado con los primeros paquetes
  if (calibrandoInicio) {
    for (int i = 0; i < CANT_FLEX; i++) calSuma[i] += p.resistencia[i];
    if (++calCant >= PAQUETES_CALIBRACION) terminarCalibracionInicio();
    return;
  }

  // Calibracion guiada: junta datos (los servos siguen moviendose)
  if (calEstado == CAL_ABIERTA) {
    for (int i = 0; i < CANT_FLEX; i++) calSuma[i] += p.resistencia[i];
    calCant++;
  } else if (calEstado == CAL_CERRADA) {
    for (int i = 0; i < CANT_FLEX; i++) {
      calMax[i] = max(calMax[i], rFiltrada[i]);
      calMin[i] = min(calMin[i], rFiltrada[i]);
    }
  }

  for (int i = 0; i < CANT_FLEX; i++) moverServoSuave(i, calcularAngulo(i, rFiltrada[i]));

  if (p.seq % HISTORIAL_CADA_N == 0 && calEstado == CAL_NO) {
    if (p.seq % (HISTORIAL_CADA_N * 40) == 0) imprimirEncabezado();
    Serial.printf("RECIBIDO %8.2f s  #%6lu |", r.t_rx_ms / 1000.0, (unsigned long)p.seq);
    for (int i = 0; i < CANT_FLEX; i++) Serial.printf(" %5lu", (unsigned long)p.resistencia[i]);
    Serial.print("       |");
    for (int i = 0; i < CANT_FLEX; i++) Serial.printf(" %3d", (int)lroundf(anguloServo[i]));
    Serial.printf("      | %6lu\n", (unsigned long)perdidos);
  }
}

// =====================================================
// COMANDOS
// =====================================================
void ejecutarComando(char *linea) {
  char c = tolower(linea[0]);
  int n = atoi(linea + 1);             // numero de flex (1 a 7), 0 = todos
  if (n < 0 || n > CANT_FLEX) { Serial.println(">>> Numero de flex invalido (1 a 7)"); return; }
  int desde = n ? n - 1 : 0, hasta = n ? n : CANT_FLEX;

  if (c == 'p' || c == 'd') {
    if (!hayDatos) { Serial.println(">>> Sin datos del guante"); return; }
    for (int i = desde; i < hasta; i++) {
      (c == 'p' ? rPlano : rDoblado)[i] = (uint32_t)rFiltrada[i];
      if (esBidireccional(i) && !otroLadoValido(i)) rOtroLado[i] = otroLadoInicial(rPlano[i]);
    }
    guardarCalibracion();
    imprimirCalibracion();
  } else if (c == 'o') {
    if (!hayDatos) { Serial.println(">>> Sin datos del guante"); return; }
    for (int i = desde; i < hasta; i++) {
      if (!esBidireccional(i)) {
        if (n) Serial.printf(">>> Flex %d no es bidireccional\n", i + 1);
        continue;
      }
      rOtroLado[i] = (uint32_t)rFiltrada[i];
      if (!otroLadoValido(i))
        Serial.printf(">>> Flex %d: el otro lado tiene que quedar al menos %d ohm por debajo del centro (%lu)\n",
                      i + 1, RANGO_MINIMO_OTRO_LADO, (unsigned long)rPlano[i]);
    }
    guardarCalibracion();
    imprimirCalibracion();
  } else if (c == 'c') {
    iniciarCalibracionGuiada();
  } else if (c == 's') {
    imprimirCalibracion();
    Serial.printf(">>> Recibidos %lu  perdidos %lu  largo distinto %lu\n", (unsigned long)recibidos,
                  (unsigned long)perdidos, (unsigned long)largoDistinto);
  } else if (c == 't' && n) {
    probarServo(n - 1);
  } else if (c == 'a' && n) {
    int plano, doblado, otro;
    int leidos = sscanf(linea + 1, "%*d %d %d %d", &plano, &doblado, &otro);
    if (leidos >= 2) {
      SERVOS[n - 1].anguloPlano = constrain(plano, 0, 180);
      SERVOS[n - 1].anguloDoblado = constrain(doblado, 0, 180);
      if (leidos == 3 && esBidireccional(n - 1)) SERVOS[n - 1].anguloOtroLado = constrain(otro, 0, 180);
      imprimirCalibracion();
    } else {
      Serial.println(">>> Uso: a3 10 170  (servo 3: estirado 10 gr, doblado 170 gr)");
      Serial.println(">>>      a7 90 170 10  (muneca: centro 90, un lado 170, otro lado 10)");
    }
  } else if (c == 'r') {
    recibidos = perdidos = largoDistinto = 0;
    Serial.println(">>> Contadores reiniciados");
  }
}

void leerComandos() {
  static char linea[32];
  static int largo = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (largo) { linea[largo] = 0; ejecutarComando(linea); largo = 0; }
    } else if (largo < (int)sizeof(linea) - 1) {
      linea[largo++] = c;
    }
  }
}

// =====================================================
// SETUP Y LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Motivo del ultimo reinicio (BROWNOUT = cayo la tension de alimentacion)
  const char *motivos[] = {"desconocido", "encendido", "externo", "software", "panico",
                           "watchdog int", "watchdog tarea", "watchdog", "deep sleep",
                           "BROWNOUT", "SDIO"};
  esp_reset_reason_t motivo = esp_reset_reason();
  Serial.printf("\nMotivo del reinicio: %s\n", motivo <= ESP_RST_SDIO ? motivos[motivo] : "?");
  Serial.println("\n=== MANO ROBOT: 7 FLEX -> 7 SERVOS (ESP32 + PCA9685) ===");

  memoria.begin("mano", false);
  cargarCalibracion();

  Wire.begin(I2C_SDA, I2C_SCL);
  pca.begin();
  pca.setPWMFreq(50);
  delay(10);

  // Los canales sin flex asignado quedan apagados (servo suelto)
  for (int canal = 0; canal < 16; canal++) {
    bool usado = false;
    for (int i = 0; i < CANT_FLEX; i++) usado |= SERVOS[i].canal == canal;
    if (!usado) pca.setPWM(canal, 0, 4096);   // 4096 = salida siempre apagada
  }

  // Lleva los servos al angulo de estirado de a uno (sin pico de corriente)
  for (int i = 0; i < CANT_FLEX; i++) {
    rFiltrada[i] = -1;
    anguloServo[i] = SERVOS[i].anguloPlano;
    escribirServo(i, anguloServo[i]);
    delay(150);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);

  Serial.printf("Mi MAC: %s  Canal: %d\n", WiFi.macAddress().c_str(), CANAL_ESPNOW);
  Serial.println("Comandos: c = calibracion guiada | p/d = estirado/doblado actual (p3 = solo flex 3)");
  Serial.println("          o7 = otro lado actual de la muneca | a7 90 170 10 = angulos muneca");
  Serial.println("          s = ver calibracion | t3 = probar servo 3 | a3 10 170 = angulos servo 3 | r = contadores");

  cola = xQueueCreate(20, sizeof(Recepcion));
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecibido);

  if (calibrandoInicio)
    Serial.println("Calibrando: mantener la mano del guante ESTIRADA y la muneca DERECHA hasta que aparezca la calibracion...");
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
  if (calEstado != CAL_NO) avanzarCalibracionGuiada();

  // Sin paquetes: los servos se quedan en la ultima posicion
  if (tUltimoRx && !sinSenalAvisado && millis() - tUltimoRx > TIMEOUT_SENAL_MS) {
    Serial.println(">>> SIN SENAL del transmisor (servos quietos en la ultima posicion)");
    sinSenalAvisado = true;
  }

  // Nunca llego un paquete: avisar cada 3 s (si no, el receptor queda en silencio)
  static uint32_t ultimoAvisoEspera = millis();
  if (!hayDatos && !largoDistinto && millis() - ultimoAvisoEspera > 3000) {
    ultimoAvisoEspera = millis();
    Serial.println(">>> ESPERANDO AL GUANTE: no llego ningun paquete. Revisar que la C3 este encendida "
                   "y tenga grabado \"5. Guante - ESP32-C3(Transmisor)\" (y no otra prueba)");
  }

  static uint32_t ultimoAvisoLargo = 0;
  if (largoDistinto && millis() - ultimoAvisoLargo > 5000) {
    ultimoAvisoLargo = millis();
    Serial.printf(">>> %lu paquetes de largo distinto: revisar que paquete.h sea igual en los dos proyectos\n",
                  (unsigned long)largoDistinto);
  }
}
