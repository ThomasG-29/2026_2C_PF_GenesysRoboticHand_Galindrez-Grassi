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
// Los flex 6 (muneca adelante) y 7 (muneca lateral) se arrastran entre si.
// La calibracion de muneca (m) mide ese acople y lo compensa: con la
// compensacion activa, los dos servos de la muneca se calculan juntos.
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
//   m      : calibracion de muneca: centro, adelante, costado del menique y
//            del pulgar (7 s cada postura). Mide y compensa el acople 6-7.
//            Con la muneca compensada, c no toca los flex 6 y 7.
//   x      : quitar la compensacion de muneca (cada flex por separado)
//   o7     : tomar la resistencia actual del flex 7 como "otro lado"
//            (solo flex bidireccionales)
//   s      : mostrar la calibracion
//   t3     : prueba del servo del flex 3 (barrido estirado -> doblado -> estirado;
//            si es bidireccional sigue -> otro lado -> centro)
//   a3 10 170 : angulos del servo 3 (estirado, doblado). Es temporal, para
//               probar: los definitivos se escriben en SERVOS[] mas abajo.
//   a7 90 170 10 : angulos del servo 7 (centro, un lado, otro lado)
//   h      : llevar la mano a HOME y dejarla quieta ahi (h otra vez: sigue al guante)
//   v      : ver la tension de los servos; v 5.12 = corregirla con lo que marca el tester
//   r      : reiniciar contadores de paquetes
//
// PROTECCION POR BAJA TENSION (ver mas abajo): mide V+ de los servos con un
// divisor en GPIO34. Tension critica -> deja de seguir al guante y va a HOME;
// tension de corte -> apaga los servos. Se rearma con h cuando se recupera.
//
// HOME: al encender la mano va a la posicion anguloHome de SERVOS[] y se queda
// ahi hasta que el guante esta conectado y calibrado (ver HOME mas abajo).
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
  uint8_t  anguloHome;      // posicion de HOME: al encender, con el comando h y (opcional) sin senal
} ServoDedo;

#define SIN_OTRO_LADO 255

// Un servo por flex, en el mismo orden (SERVOS[0] lo mueve el flex 1)
//                                                   estirado doblado otro lado   HOME
ServoDedo SERVOS[CANT_FLEX] = {
  {"S1 MG90S",    0, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO,  10},   // Flex 1
  {"S2 MG90S",    1, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO,  10},   // Flex 2
  {"S3 MG90S",    2, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO,  10},   // Flex 3
  {"S4 MG90S",    3, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO,  10},   // Flex 4
  {"S5 MG90S",    4, MG90S_MIN_US,  MG90S_MAX_US,  10, 170, SIN_OTRO_LADO,  10},   // Flex 5
  {"S11 DS3225", 10, DS3225_MIN_US, DS3225_MAX_US, 10, 170, SIN_OTRO_LADO,  10},   // Flex 6
  {"S12 DS3225", 11, DS3225_MIN_US, DS3225_MAX_US, 90, 170, 10,             90},   // Flex 7 (muneca lateral)
};

bool esBidireccional(int i) { return SERVOS[i].anguloOtroLado != SIN_OTRO_LADO; }

// =====================================================
// MOVIMIENTO CONTINUO DE LOS SERVOS
// =====================================================
// El guante manda 25 paquetes por segundo y el conversor (ADS) mide la
// resistencia de a ~12 ohm, que en un dedo son 3 a 7 grados: si el servo se
// moviera directo a cada valor que llega, avanzaria a saltos. Por eso cada
// paquete solo fija el OBJETIVO, y el servo se acerca a el 100 veces por
// segundo con un suavizado (los saltos se convierten en rampas).
#define PERIODO_SERVOS_MS 10       // actualizacion de los servos (100 por segundo)
#define SUAVIZADO_SERVOS  0.20f    // fraccion de la distancia al objetivo por actualizacion
                                   // (mas chico = mas suave pero mas lento; 0.20 -> ~45 ms)
#define VELOCIDAD_MAXIMA  250.0f   // grados por segundo
// Zona muerta con histeresis: el servo arranca si el objetivo se aleja mas de
// ARRANQUE y para cuando queda a menos de PARADA (asi el ruido de la medicion
// no lo hace temblar quieto)
#define ZONA_ARRANQUE 1.5f         // grados
#define ZONA_PARADA   0.3f         // grados

// =====================================================
// HOME
// =====================================================
// Al encender, la mano va a HOME y se queda ahi hasta que el guante esta
// conectado y calibrado; recien ahi empieza a seguirlo (a velocidad limitada).
// El comando h la lleva a HOME y la deja quieta ahi (h otra vez: sigue al guante).
#define VELOCIDAD_HOME     60.0f   // grados por segundo yendo a HOME (despacio)
#define ESPERA_HOME_MS     1500    // tiempo en HOME antes de empezar a seguir al guante
#define VOLVER_HOME_SIN_SENAL 0    // 1: si se pierde el guante, la mano vuelve a HOME
                                   // 0: se queda quieta en la ultima posicion

// Suavizado de la resistencia: mas chico = mas suave pero mas lento
#define ALPHA 0.4f

#define I2C_SDA 21
#define I2C_SCL 22

// =====================================================
// PROTECCION POR BAJA TENSION DE LOS SERVOS
// =====================================================
// Se mide la tension V+ de los servos con el modulo "sensor de voltaje 0-25 V"
// (divisor de 30k y 7.5k), conectado en paralelo, sin cortar nada:
//   bornera VCC del modulo -> pin V+ de la tira de pines del PCA9685
//   bornera GND del modulo -> GND
//   pin S del modulo       -> GPIO34 (ADC1: el ADC2 no funciona con WiFi/ESP-NOW)
// Con dos resistencias sueltas (10k de V+ a GPIO34 y 4.7k de GPIO34 a GND)
// cambiar DIVISOR_TENSION por ((10000.0f + 4700.0f) / 4700.0f).
//
// Tensiones de los servos (hoja de datos): MG90S y MG92B 4.8 a 6.0 V,
// DS3225 4.8 a 6.8 V. Por debajo de 4.8 V pierden fuerza; los DS3225 son
// digitales y alrededor de 4.0-4.5 V su microcontrolador se puede reiniciar
// y el servo salta a cualquier lado. El ESP32 tambien se alimenta de esta
// fuente (VIN -> regulador 3.3 V, que necesita ~4.3 V en VIN).
#define PIN_TENSION        34
#define DIVISOR_TENSION    ((30000.0f + 7500.0f) / 7500.0f)   // modulo 0-25 V: divide por 5
#define TENSION_AVISO      4.6f   // V: aviso por el monitor (todavia sigue al guante)
#define TENSION_CRITICA    4.3f   // V: deja de seguir al guante y va a HOME (mano abierta: suelta el objeto)
#define TENSION_CORTE      3.9f   // V: apaga los servos (quedan sueltos, sin fuerza) para que no salten
#define TENSION_RECUPERADA 4.8f   // V: por encima de esto se puede rearmar con h
#define TIEMPO_CRITICO_MS  150    // tiene que estar por debajo este tiempo seguido (los picos de
                                  // un instante al arrancar un servo no cuentan)
#define TIEMPO_RECUPERADA_MS 3000 // tiene que estar recuperada este tiempo seguido para rearmar
#define TENSION_SIN_DIVISOR 2.0f  // V: si al encender se lee menos, el divisor no esta conectado

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
#define RANGO_MINIMO   150     // ohms: si un flex cambia menos, no se calibra
                               // (con el guante puesto el pulgar y el anular cambian ~250)
#define MARGEN_DOBLADO 0.05f   // el doblado queda un 5% antes del maximo medido

// Flex bidireccional (muneca): hacia el lado donde BAJA la resistencia
// el flex cambia bastante menos, por eso el rango y el minimo son menores
#define RANGO_INICIAL_OTRO_LADO 500   // ohms por debajo del centro si nunca se calibro
#define RANGO_MINIMO_OTRO_LADO  100   // ohms: si cambia menos, no se calibra ese lado
// El flex no vuelve siempre al mismo valor en el centro (+-60 ohm, y despues
// de doblarlo queda unos segundos ~150 ohm por debajo): zona muerta alrededor
// del centro donde el servo queda quieto en el centro
#define ZONA_CENTRO 0.15f             // 15% del recorrido hacia el lado donde sube
#define ZONA_CENTRO_OTRO_LADO 0.45f   // hacia el otro lado cambia menos: zona mas grande

// =====================================================
// COMPENSACION DE LA MUNECA (acople entre flex 6 y 7)
// =====================================================
// Los dos flex de la muneca se arrastran entre si: al doblarla hacia
// adelante el flex 7 tambien sube, y al llevarla a un costado el 6 tambien.
// La calibracion de muneca (comando m) mide cuanto cambia CADA flex en cada
// postura (adelante, un costado, el otro) y con eso se separan los dos
// movimientos resolviendo un sistema de 2x2:
//   cambio R6 = f * adelante.R6 + s * costado.R6
//   cambio R7 = f * adelante.R7 + s * costado.R7
//   f = cuanto esta doblada hacia adelante (0..1), s = costado (-1..1)
#define FLEX_MUNECA_ADELANTE 5   // flex 6 (indice 5)
#define FLEX_MUNECA_LATERAL  6   // flex 7 (indice 6)

// El flex 6 tampoco vuelve siempre al mismo valor (+~90 ohm despues de doblarlo)
#define ZONA_CENTRO_ADELANTE 0.20f
// Cuanto R7 sube al doblar hacia adelante varia (+220 a +410 ohm), y la
// diferencia parece un costado: con la muneca hacia adelante la zona muerta
// del costado crece hasta este porcentaje del recorrido que queda
#define ZONA_LATERAL_EXTRA_ADELANTE 0.60f

// Calibracion de muneca: cada postura dura CAL_POSTURA_MS y se promedian los
// valores despues de CAL_DESCARTE_MS (el tiempo de llegar a la postura)
// (leer la indicacion en voz alta tarda ~3 s: se promedian los ultimos 3 s)
#define CAL_POSTURA_MS  7000
#define CAL_CENTRO_MS   6000
#define CAL_DESCARTE_MS 4000

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
float anguloServo[CANT_FLEX];     // angulo que tiene cada servo ahora
float objetivoServo[CANT_FLEX];   // angulo al que tiene que ir (segun el guante)
bool hayObjetivo = false;
uint32_t ultimaR[CANT_FLEX];
bool hayDatos = false;

// HOME: mientras enHome los servos van a anguloHome e ignoran al guante
bool enHome = true;          // al encender arranca en HOME
bool homeManual = false;     // comando h: se queda en HOME hasta otro h
uint32_t tEnHome = 0;        // cuando llegaron todos los servos a HOME (0 = todavia no)

// Proteccion por baja tension
float tension = 0;              // V+ de los servos (filtrada)
float factorTension = 1.0f;     // correccion por la tolerancia de las resistencias (comando v)
bool divisorConectado = false;  // se decide al encender
bool fallaTension = false;      // tension critica: en HOME hasta rearmar con h
bool servosApagados = false;    // tension de corte: PWM apagado (servos sueltos)
bool avisoTension = false;

uint32_t recibidos = 0, perdidos = 0, ultimoSeq = 0, tUltimoRx = 0;
volatile uint32_t largoDistinto = 0;
bool sinSenalAvisado = false;

// Calibracion del estirado al inicio
bool calibrandoInicio = CALIBRAR_PLANO_AL_INICIO;
uint32_t calCant = 0;
uint64_t calSuma[CANT_FLEX];

// Calibracion guiada
enum { CAL_NO, CAL_ABIERTA, CAL_CERRADA, CAL_MUNECA } calEstado = CAL_NO;
uint32_t calInicio = 0, calUltimoAviso = 0;
float calMax[CANT_FLEX], calMin[CANT_FLEX];

// Compensacion de la muneca: cambio del OTRO flex en cada postura (ohms).
// El cambio del flex propio sale de rDoblado/rOtroLado - rPlano.
typedef struct {
  int32_t r7Adelante;   // cambio de R7 con la muneca hacia adelante
  int32_t r6Lado;       // cambio de R6 con la muneca hacia el lado donde SUBE R7
  int32_t r6OtroLado;   // cambio de R6 con la muneca hacia el lado donde BAJA R7
} AcopleMuneca;
AcopleMuneca acople = {0, 0, 0};
bool acopleCalibrado = false;

// Calibracion de muneca (comando m): una fase por postura
enum { POSTURA_CENTRO, POSTURA_ADELANTE, POSTURA_LADO1, POSTURA_LADO2, CANT_POSTURAS };
typedef struct {
  uint8_t postura;
  uint16_t duracionMs;
  const char *aviso;    // lo que se imprime (y lee en voz alta el script de calibracion)
} FaseMuneca;
const FaseMuneca FASES_MUNECA[] = {
  {POSTURA_CENTRO,   CAL_CENTRO_MS,  "Muneca derecha y quieta"},
  {POSTURA_ADELANTE, CAL_POSTURA_MS, "Muneca hacia adelante, al maximo, y mantener"},
  {POSTURA_CENTRO,   CAL_CENTRO_MS,  "Volver al centro"},
  {POSTURA_LADO1,    CAL_POSTURA_MS, "Muneca al costado del menique, al maximo, y mantener"},
  {POSTURA_CENTRO,   CAL_CENTRO_MS,  "Volver al centro"},
  {POSTURA_LADO2,    CAL_POSTURA_MS, "Muneca al costado del pulgar, al maximo, y mantener"},
  {POSTURA_CENTRO,   CAL_CENTRO_MS,  "Volver al centro"},
};
const int CANT_FASES_MUNECA = sizeof(FASES_MUNECA) / sizeof(FASES_MUNECA[0]);
int faseMuneca = 0;
float munSuma6[CANT_POSTURAS], munSuma7[CANT_POSTURAS];
uint32_t munCant[CANT_POSTURAS];

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
  if (servosApagados) return;   // corte por baja tension: nada mueve los servos
  const ServoDedo &s = SERVOS[i];
  angulo = constrain(angulo, 0.0f, 180.0f);
  float us = s.minUs + angulo * (s.maxUs - s.minUs) / 180.0f;
  pca.setPWM(s.canal, 0, (uint16_t)lroundf(us * 4096.0f / 20000.0f));
}

// =====================================================
// HOME
// =====================================================
void irAHome(bool manual) {
  if (!enHome) Serial.println(manual ? ">>> HOME: la mano va a HOME y se queda ahi (h para seguir al guante)"
                                     : ">>> HOME: la mano vuelve a HOME");
  enHome = true;
  homeManual = manual;
  tEnHome = 0;
}

// Se llama seguido desde loop(): mueve los servos hacia HOME a VELOCIDAD_HOME
// y, cuando corresponde, devuelve el control al guante
void actualizarHome() {
  static uint32_t tAnterior = millis();
  uint32_t ahora = millis();
  float paso = VELOCIDAD_HOME * (ahora - tAnterior) / 1000.0f;
  tAnterior = ahora;
  if (!enHome) return;

  bool todosEnHome = true;
  for (int i = 0; i < CANT_FLEX; i++) {
    float dif = SERVOS[i].anguloHome - anguloServo[i];
    if (fabsf(dif) < 0.5f) continue;
    todosEnHome = false;
    anguloServo[i] += constrain(dif, -paso, paso);
    escribirServo(i, anguloServo[i]);
  }
  if (!todosEnHome) { tEnHome = 0; return; }
  if (tEnHome == 0) {
    tEnHome = ahora;
    Serial.println(">>> HOME: mano en posicion de HOME");
  }

  // Sigue al guante solo si hay senal, ya se calibro el inicio y paso la espera
  // (con falla de tension se queda en HOME hasta rearmar con h)
  bool guanteListo = hayDatos && !calibrandoInicio && !sinSenalAvisado;
  if (!homeManual && !fallaTension && guanteListo && ahora - tEnHome >= ESPERA_HOME_MS) {
    enHome = false;
    Serial.println(">>> Siguiendo al guante");
  }
}

// =====================================================
// PROTECCION POR BAJA TENSION
// =====================================================
float leerTension() {
  return analogReadMilliVolts(PIN_TENSION) / 1000.0f * DIVISOR_TENSION * factorTension;
}

void apagarServos() {
  servosApagados = true;
  for (int i = 0; i < CANT_FLEX; i++) pca.setPWM(SERVOS[i].canal, 0, 4096);   // sin pulso: servo suelto
}

// Vuelve a encender los servos llevandolos a HOME de a uno (no se sabe donde
// quedaron mientras estuvieron sueltos)
void encenderServosEnHome() {
  servosApagados = false;
  for (int i = 0; i < CANT_FLEX; i++) {
    anguloServo[i] = SERVOS[i].anguloHome;
    escribirServo(i, anguloServo[i]);
    delay(150);
  }
  tUltimoRx = millis();   // la pausa no cuenta como SIN SENAL
}

// Comando h con falla de tension: solo rearma si la tension se recupero
bool rearmarTension() {
  if (tension < TENSION_RECUPERADA) {
    Serial.printf(">>> TENSION: %.2f V, todavia baja (hace falta mas de %.1f V). No se rearma.\n",
                  tension, TENSION_RECUPERADA);
    return false;
  }
  if (servosApagados) encenderServosEnHome();
  fallaTension = false;
  Serial.printf(">>> TENSION: rearmado con %.2f V\n", tension);
  return true;
}

// Se llama seguido desde loop(): mide V+ (filtrada) y actua por niveles
void revisarTension() {
  static uint32_t tMedicion = 0, tBajoCritica = 0, tBajoCorte = 0, tRecuperada = 0, tAviso = 0;
  static bool avisoRecuperada = false;
  uint32_t ahora = millis();
  if (ahora - tMedicion < 5) return;
  tMedicion = ahora;

  float v = leerTension();
  tension = tension == 0 ? v : tension + 0.3f * (v - tension);   // filtro: ~15 ms
  if (!divisorConectado) {
    if (ahora - tAviso > 10000) {
      tAviso = ahora;
      Serial.printf(">>> TENSION: no se detecta el divisor en GPIO%d (%.2f V): PROTECCION DESACTIVADA\n",
                    PIN_TENSION, tension);
    }
    return;
  }

  // Corte: servos sueltos (con tan poca tension pueden saltar a cualquier lado)
  if (tension < TENSION_CORTE) {
    if (!tBajoCorte) tBajoCorte = ahora;
    if (!servosApagados && ahora - tBajoCorte >= TIEMPO_CRITICO_MS) {
      apagarServos();
      fallaTension = true;
      irAHome(false);
      Serial.printf(">>> TENSION DE CORTE: %.2f V (< %.1f V). SERVOS APAGADOS (sueltos, sin fuerza).\n",
                    tension, TENSION_CORTE);
      Serial.println(">>> Revisar la fuente. Con la tension recuperada, escribir h para rearmar.");
    }
  } else {
    tBajoCorte = 0;
  }

  // Critica: deja de seguir al guante y va a HOME (mano abierta: suelta el objeto)
  if (tension < TENSION_CRITICA) {
    if (!tBajoCritica) tBajoCritica = ahora;
    if (!fallaTension && ahora - tBajoCritica >= TIEMPO_CRITICO_MS) {
      fallaTension = true;
      irAHome(false);
      Serial.printf(">>> TENSION CRITICA: %.2f V (< %.1f V). La mano deja de seguir al guante y va a HOME.\n",
                    tension, TENSION_CRITICA);
      Serial.println(">>> Revisar la fuente. Con la tension recuperada, escribir h para rearmar.");
    }
  } else {
    tBajoCritica = 0;
  }

  // Aviso (con histeresis para no repetirlo)
  if (!avisoTension && tension < TENSION_AVISO) {
    avisoTension = true;
    Serial.printf(">>> TENSION BAJA: %.2f V (los servos pierden fuerza por debajo de 4.8 V)\n", tension);
  } else if (avisoTension && tension > TENSION_AVISO + 0.1f) {
    avisoTension = false;
  }

  // En falla: avisa una vez cuando ya se puede rearmar
  if (fallaTension && tension >= TENSION_RECUPERADA) {
    if (!tRecuperada) tRecuperada = ahora;
    if (!avisoRecuperada && ahora - tRecuperada >= TIEMPO_RECUPERADA_MS) {
      avisoRecuperada = true;
      Serial.printf(">>> TENSION recuperada (%.2f V): escribir h para rearmar\n", tension);
    }
  } else {
    tRecuperada = 0;
    avisoRecuperada = false;
  }
}

// Cada paquete del guante solo fija hacia donde tiene que ir el servo
void fijarObjetivo(int i, float angulo) {
  objetivoServo[i] = angulo;
  hayObjetivo = true;
}

// Se llama seguido desde loop(): acerca cada servo a su objetivo de forma
// continua (PERIODO_SERVOS_MS), con suavizado y velocidad maxima
void actualizarServos() {
  static uint32_t tAnterior = 0;
  static bool moviendo[CANT_FLEX];
  uint32_t ahora = millis();
  if (ahora - tAnterior < PERIODO_SERVOS_MS) return;
  float dt = min(ahora - tAnterior, (uint32_t)50) / 1000.0f;
  tAnterior = ahora;
  if (enHome || !hayObjetivo) {               // en HOME manda actualizarHome()
    for (int i = 0; i < CANT_FLEX; i++) moviendo[i] = false;
    return;
  }

  float pasoMax = VELOCIDAD_MAXIMA * dt;
  for (int i = 0; i < CANT_FLEX; i++) {
    float dif = objetivoServo[i] - anguloServo[i];
    if (!moviendo[i] && fabsf(dif) < ZONA_ARRANQUE) continue;
    moviendo[i] = fabsf(dif) >= ZONA_PARADA;
    if (!moviendo[i]) continue;
    anguloServo[i] += constrain(dif * SUAVIZADO_SERVOS, -pasoMax, pasoMax);
    escribirServo(i, anguloServo[i]);
  }
}

// Bidireccional: cerca del centro el servo queda quieto en el centro
// (el ruido de la muneca derecha no lo hace cabecear de un lado al otro)
float aplicarZonaCentro(float t, float zona = ZONA_CENTRO) {
  return t <= zona ? 0.0f : (t - zona) / (1.0f - zona);
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
      t = aplicarZonaCentro(constrain(t, 0.0f, 1.0f), ZONA_CENTRO_OTRO_LADO);
      return s.anguloPlano + t * (s.anguloOtroLado - s.anguloPlano);
    }
    t = aplicarZonaCentro(constrain(t, 0.0f, 1.0f));
  }

  t = constrain(t, 0.0f, 1.0f);
  return s.anguloPlano + t * (s.anguloDoblado - s.anguloPlano);
}

// Muneca con compensacion: separa "adelante" (f) y "costado" (s) a partir de
// R6 y R7 juntos (ver COMPENSACION DE LA MUNECA arriba) y da los dos angulos
void calcularMuneca(float r6, float r7, float &angulo6, float &angulo7) {
  const int A = FLEX_MUNECA_ADELANTE, L = FLEX_MUNECA_LATERAL;
  // Cambio respecto del centro, y cambio de cada flex en cada postura
  float x6 = r6 - (float)rPlano[A], x7 = r7 - (float)rPlano[L];
  // R6 solo sube al doblar la muneca: si baja (por ejemplo, al cerrar el puno
  // el guante tira del flex) no es un costado, se toma como centro
  if (x6 < 0) x6 = 0;
  float adel6 = (float)rDoblado[A] - rPlano[A], adel7 = acople.r7Adelante;
  float lado6 = acople.r6Lado,     lado7 = (float)rDoblado[L] - rPlano[L];
  float otro6 = acople.r6OtroLado, otro7 = (float)rOtroLado[L] - rPlano[L];

  float f = 0, s = 0;
  bool resuelto = false;
  // Primero se prueba hacia el lado donde sube R7...
  float det = adel6 * lado7 - lado6 * adel7;
  if (fabsf(det) > 1.0f) {
    s = (adel6 * x7 - adel7 * x6) / det;
    f = (x6 * lado7 - lado6 * x7) / det;
    resuelto = s >= 0;
  }
  // ...y si da negativo, hacia el otro lado
  if (!resuelto && esBidireccional(L)) {
    det = adel6 * otro7 - otro6 * adel7;
    if (fabsf(det) > 1.0f) {
      float so = (adel6 * x7 - adel7 * x6) / det;
      if (so >= 0) {
        f = (x6 * otro7 - otro6 * x7) / det;
        s = -so;
        resuelto = true;
      }
    }
  }
  // Ninguno cierra (muneca casi en el centro): solo adelante
  if (!resuelto) {
    s = 0;
    f = adel6 != 0 ? x6 / adel6 : 0;
  }

  const ServoDedo &sa = SERVOS[A], &sl = SERVOS[L];
  f = constrain(f, 0.0f, 1.0f);
  angulo6 = sa.anguloPlano + aplicarZonaCentro(f, ZONA_CENTRO_ADELANTE) * (sa.anguloDoblado - sa.anguloPlano);
  float extra = ZONA_LATERAL_EXTRA_ADELANTE * f;   // muneca hacia adelante: costado menos sensible
  if (s >= 0) {
    float zona = ZONA_CENTRO + (1.0f - ZONA_CENTRO) * extra;
    float t = aplicarZonaCentro(constrain(s, 0.0f, 1.0f), zona);
    angulo7 = sl.anguloPlano + t * (sl.anguloDoblado - sl.anguloPlano);
  } else {
    float zona = ZONA_CENTRO_OTRO_LADO + (1.0f - ZONA_CENTRO_OTRO_LADO) * extra;
    float t = aplicarZonaCentro(constrain(-s, 0.0f, 1.0f), zona);
    angulo7 = sl.anguloPlano + t * (sl.anguloOtroLado - sl.anguloPlano);
  }
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
  memoria.putBytes("acople", &acople, sizeof(acople));
  memoria.putBool("acopleOk", acopleCalibrado);
  calibracionGuardada = true;
}

// Flex de la muneca con la compensacion calibrada (se manejan juntos)
bool esMunecaCompensada(int i) {
  return acopleCalibrado && (i == FLEX_MUNECA_ADELANTE || i == FLEX_MUNECA_LATERAL);
}

// Mueve el centro de un flex sin cambiar cuanto cambia en cada postura
void moverCentro(int i, uint32_t nuevo) {
  int32_t dif = (int32_t)nuevo - (int32_t)rPlano[i];
  rPlano[i] = nuevo;
  rDoblado[i] += dif;
  rOtroLado[i] += dif;
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
    if (memoria.getBool("acopleOk", false) && memoria.getBytesLength("acople") == sizeof(acople)) {
      memoria.getBytes("acople", &acople, sizeof(acople));
      acopleCalibrado = true;
    }
  }
  Serial.println(ok ? "Calibracion cargada de la memoria." : "Sin calibracion guardada: valores por defecto.");
  if (ok && !acopleCalibrado) Serial.println("Muneca sin compensar: hacer la calibracion de muneca (m).");
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
  if (acopleCalibrado)
    Serial.printf(">>> Muneca compensada: adelante mueve R7 %+ld | costados mueven R6 %+ld / %+ld ohm\n",
                  (long)acople.r7Adelante, (long)acople.r6Lado, (long)acople.r6OtroLado);
  else
    Serial.println(">>> Muneca SIN compensar (los flex 6 y 7 se arrastran): usar el comando m");
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
    // Ya calibrado: el doblado y el otro lado se corren junto con el estirado
    // (el flex no vuelve siempre al mismo valor, pero cuanto cambia al doblarlo
    // si se mantiene)
    if (calibracionGuardada) {
      moverCentro(i, medido);
      continue;
    }
    // Nunca se calibro: estirado medido + rangos tipicos
    rPlano[i] = medido;
    rDoblado[i] = rPlano[i] + RANGO_INICIAL;
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
      if (esMunecaCompensada(i)) {
        Serial.printf(">>> Flex %d (muneca): se calibra aparte con el comando m\n", i + 1);
        continue;
      }
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
// CALIBRACION DE MUNECA (comando m)
// =====================================================
void avisarFaseMuneca() {
  Serial.printf(">>> MUNECA %d/%d: %s\n", faseMuneca + 1, CANT_FASES_MUNECA, FASES_MUNECA[faseMuneca].aviso);
}

void iniciarCalibracionMuneca() {
  if (!hayDatos || calibrandoInicio) { Serial.println(">>> Sin datos del guante: no se puede calibrar"); return; }
  for (int k = 0; k < CANT_POSTURAS; k++) munSuma6[k] = munSuma7[k] = munCant[k] = 0;
  faseMuneca = 0;
  calEstado = CAL_MUNECA;
  calInicio = millis();
  Serial.println("\n>>> CALIBRACION DE MUNECA: mano abierta, seguir las indicaciones");
  avisarFaseMuneca();
}

// Junta datos de la fase actual (solo despues de llegar a la postura)
void juntarDatosMuneca(const PaqueteGuante &p) {
  if (millis() - calInicio < CAL_DESCARTE_MS) return;
  uint8_t k = FASES_MUNECA[faseMuneca].postura;
  munSuma6[k] += p.resistencia[FLEX_MUNECA_ADELANTE];
  munSuma7[k] += p.resistencia[FLEX_MUNECA_LATERAL];
  munCant[k]++;
}

void terminarCalibracionMuneca() {
  const int A = FLEX_MUNECA_ADELANTE, L = FLEX_MUNECA_LATERAL;
  calEstado = CAL_NO;
  for (int k = 0; k < CANT_POSTURAS; k++)
    if (munCant[k] == 0) { Serial.println(">>> Sin datos: calibracion de muneca cancelada"); return; }

  // Promedio de cada postura (el centro promedia todas las vueltas al centro:
  // el flex no vuelve siempre al mismo valor)
  float c6 = munSuma6[POSTURA_CENTRO] / munCant[POSTURA_CENTRO];
  float c7 = munSuma7[POSTURA_CENTRO] / munCant[POSTURA_CENTRO];
  float d6[CANT_POSTURAS], d7[CANT_POSTURAS];   // cambio respecto del centro
  for (int k = 0; k < CANT_POSTURAS; k++) {
    d6[k] = munSuma6[k] / munCant[k] - c6;
    d7[k] = munSuma7[k] / munCant[k] - c7;
  }
  Serial.println(">>> Cambio respecto del centro (ohm):    R6      R7");
  const char *nombres[] = {"centro", "adelante", "menique", "pulgar"};
  for (int k = 1; k < CANT_POSTURAS; k++)
    Serial.printf(">>>   %-10s                       %+6.0f  %+6.0f\n", nombres[k], d6[k], d7[k]);

  // El lado "doblado" del flex 7 es donde SUBE R7 (no importa si es menique o pulgar)
  int lado = d7[POSTURA_LADO1] >= d7[POSTURA_LADO2] ? POSTURA_LADO1 : POSTURA_LADO2;
  int otro = lado == POSTURA_LADO1 ? POSTURA_LADO2 : POSTURA_LADO1;

  if (d6[POSTURA_ADELANTE] < 2 * RANGO_MINIMO) {
    Serial.printf(">>> El flex 6 casi no cambio hacia adelante (%.0f ohm): calibracion cancelada\n", d6[POSTURA_ADELANTE]);
    return;
  }
  if (d7[lado] < 2 * RANGO_MINIMO) {
    Serial.printf(">>> El flex 7 casi no cambio hacia los costados (%.0f ohm): calibracion cancelada\n", d7[lado]);
    return;
  }
  float det = d6[POSTURA_ADELANTE] * d7[lado] - d6[lado] * d7[POSTURA_ADELANTE];
  if (det < 0.2f * d6[POSTURA_ADELANTE] * d7[lado]) {
    Serial.println(">>> Adelante y costado mueven los dos flex casi igual: no se pueden separar.");
    Serial.println(">>> Revisar la colocacion de los flex 6 y 7. Calibracion cancelada.");
    return;
  }

  // El doblado queda un poco antes del maximo (igual que en la calibracion guiada)
  float m = 1.0f - MARGEN_DOBLADO;
  rPlano[A] = (uint32_t)c6;
  rPlano[L] = (uint32_t)c7;
  rDoblado[A] = (uint32_t)(c6 + m * d6[POSTURA_ADELANTE]);
  rDoblado[L] = (uint32_t)(c7 + m * d7[lado]);
  acople.r7Adelante = (int32_t)(m * d7[POSTURA_ADELANTE]);
  acople.r6Lado = (int32_t)(m * d6[lado]);
  if (-d7[otro] >= RANGO_MINIMO_OTRO_LADO) {
    rOtroLado[L] = (uint32_t)(c7 + m * d7[otro]);
    acople.r6OtroLado = (int32_t)(m * d6[otro]);
  } else {
    Serial.printf(">>> Hacia el %s el flex 7 casi no cambio (%+.0f ohm): ese lado queda con el valor por defecto\n",
                  nombres[otro], d7[otro]);
    rOtroLado[L] = otroLadoInicial(rPlano[L]);
    acople.r6OtroLado = 0;
  }
  Serial.printf(">>> El flex 7 sube hacia el %s: ese lado va a %d gr, el otro a %d gr (si queda al reves: a7 %d %d %d)\n",
                nombres[lado], SERVOS[L].anguloDoblado, SERVOS[L].anguloOtroLado,
                SERVOS[L].anguloPlano, SERVOS[L].anguloOtroLado, SERVOS[L].anguloDoblado);
  acopleCalibrado = true;
  guardarCalibracion();
  Serial.println(">>> Calibracion de muneca terminada y guardada.");
  imprimirCalibracion();
  imprimirEncabezado();
}

void avanzarCalibracionMuneca() {
  if (millis() - calInicio < FASES_MUNECA[faseMuneca].duracionMs) return;
  if (++faseMuneca >= CANT_FASES_MUNECA) { terminarCalibracionMuneca(); return; }
  calInicio = millis();
  avisarFaseMuneca();
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
  } else if (calEstado == CAL_MUNECA) {
    juntarDatosMuneca(p);
  }

  // Objetivo de cada servo (se calcula siempre, aun en HOME, para que al
  // salir de HOME el servo vaya directo a donde esta el guante)
  for (int i = 0; i < CANT_FLEX; i++) {
    if (esMunecaCompensada(i)) continue;
    fijarObjetivo(i, calcularAngulo(i, rFiltrada[i]));
  }
  if (acopleCalibrado) {
    float a6, a7;
    calcularMuneca(rFiltrada[FLEX_MUNECA_ADELANTE], rFiltrada[FLEX_MUNECA_LATERAL], a6, a7);
    fijarObjetivo(FLEX_MUNECA_ADELANTE, a6);
    fijarObjetivo(FLEX_MUNECA_LATERAL, a7);
  }

  if (p.seq % HISTORIAL_CADA_N == 0 && calEstado == CAL_NO) {
    if (p.seq % (HISTORIAL_CADA_N * 40) == 0) imprimirEncabezado();
    Serial.printf("RECIBIDO %8.2f s  #%6lu |", r.t_rx_ms / 1000.0, (unsigned long)p.seq);
    for (int i = 0; i < CANT_FLEX; i++) Serial.printf(" %5lu", (unsigned long)p.resistencia[i]);
    Serial.print("       |");
    for (int i = 0; i < CANT_FLEX; i++) Serial.printf(" %3d", (int)lroundf(anguloServo[i]));
    Serial.printf("      | %6lu", (unsigned long)perdidos);
    if (divisorConectado) Serial.printf(" | %4.2f V", tension);
    Serial.println();
  }
}

// Comando v: muestra la tension; "v 5.12" corrige la medicion con lo que marca
// el tester en la bornera V+ del PCA9685 (se guarda en la memoria)
void comandoTension(const char *args) {
  float real = atof(args);
  if (real > 0) {
    float medida = 0;
    for (int k = 0; k < 20; k++) { medida += leerTension(); delay(2); }
    medida /= 20;
    if (medida < TENSION_SIN_DIVISOR) { Serial.println(">>> TENSION: no se detecta el divisor"); return; }
    factorTension *= real / medida;
    memoria.putFloat("factorV", factorTension);
    tension = real;
    Serial.printf(">>> TENSION: corregida (factor %.3f) y guardada\n", factorTension);
  }
  Serial.printf(">>> TENSION servos: %.2f V | aviso < %.1f | critica < %.1f (HOME) | corte < %.1f (servos sueltos)\n",
                tension, TENSION_AVISO, TENSION_CRITICA, TENSION_CORTE);
  Serial.printf(">>> Proteccion: %s%s%s\n", divisorConectado ? "ACTIVA" : "DESACTIVADA (sin divisor)",
                fallaTension ? " | EN FALLA (h para rearmar)" : "", servosApagados ? " | SERVOS APAGADOS" : "");
}

// =====================================================
// COMANDOS
// =====================================================
void ejecutarComando(char *linea) {
  char c = tolower(linea[0]);
  if (c == 'v') { comandoTension(linea + 1); return; }
  int n = atoi(linea + 1);             // numero de flex (1 a 7), 0 = todos
  if (n < 0 || n > CANT_FLEX) { Serial.println(">>> Numero de flex invalido (1 a 7)"); return; }
  int desde = n ? n - 1 : 0, hasta = n ? n : CANT_FLEX;

  if (c == 'p' || c == 'd') {
    if (!hayDatos) { Serial.println(">>> Sin datos del guante"); return; }
    for (int i = desde; i < hasta; i++) {
      if (esMunecaCompensada(i)) {
        // Muneca compensada: p corre el centro, el doblado se calibra con m
        if (c == 'p') moverCentro(i, (uint32_t)rFiltrada[i]);
        else if (n) Serial.printf(">>> Flex %d (muneca): el doblado se calibra con el comando m\n", i + 1);
        continue;
      }
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
      if (esMunecaCompensada(i)) {
        Serial.printf(">>> Flex %d (muneca): el otro lado se calibra con el comando m\n", i + 1);
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
  } else if (c == 'h') {
    if (fallaTension) {
      // Rearme despues de una falla de tension: vuelve a seguir al guante
      if (rearmarTension()) homeManual = false;
    } else if (homeManual) {
      homeManual = false;   // actualizarHome() devuelve el control al guante
      Serial.println(">>> HOME: vuelve a seguir al guante");
    } else {
      irAHome(true);
    }
  } else if (c == 'm') {
    iniciarCalibracionMuneca();
  } else if (c == 'x') {
    acopleCalibrado = false;   // vuelve a mover cada flex de la muneca por separado
    guardarCalibracion();
    imprimirCalibracion();
  } else if (c == 's') {
    imprimirCalibracion();
    Serial.printf(">>> Recibidos %lu  perdidos %lu  largo distinto %lu\n", (unsigned long)recibidos,
                  (unsigned long)perdidos, (unsigned long)largoDistinto);
    comandoTension("");
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
  // Buffer grande: imprimir una linea no frena el programa mientras sale por
  // el USB (a 115200 baudios cada linea del historial tarda ~12 ms)
  Serial.setTxBufferSize(2048);
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

  // Proteccion por baja tension: si no se lee nada, el divisor no esta conectado
  analogSetPinAttenuation(PIN_TENSION, ADC_11db);   // rango hasta ~3.1 V en el pin
  factorTension = memoria.getFloat("factorV", 1.0f);
  for (int k = 0; k < 20; k++) { tension += leerTension(); delay(2); }
  tension /= 20;
  divisorConectado = tension >= TENSION_SIN_DIVISOR;
  if (divisorConectado)
    Serial.printf("Tension de los servos: %.2f V (proteccion: critica < %.1f V -> HOME, corte < %.1f V)\n",
                  tension, TENSION_CRITICA, TENSION_CORTE);
  else
    Serial.printf("ATENCION: no se detecta el divisor de tension en GPIO%d: PROTECCION DESACTIVADA\n", PIN_TENSION);

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

  // Lleva los servos a HOME de a uno (sin pico de corriente). El servo no
  // informa donde esta, asi que este primer movimiento es a su velocidad maxima;
  // los siguientes (h, sin senal) van despacio a VELOCIDAD_HOME
  for (int i = 0; i < CANT_FLEX; i++) {
    rFiltrada[i] = -1;
    anguloServo[i] = SERVOS[i].anguloHome;
    escribirServo(i, anguloServo[i]);
    delay(150);
  }
  tEnHome = millis();
  Serial.println(">>> HOME: mano en posicion de HOME (sigue al guante cuando este conectado y calibrado)");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);

  Serial.printf("Mi MAC: %s  Canal: %d\n", WiFi.macAddress().c_str(), CANAL_ESPNOW);
  Serial.println("Comandos: c = calibracion guiada | p/d = estirado/doblado actual (p3 = solo flex 3)");
  Serial.println("          m = calibracion de muneca (flex 6 y 7 juntos) | x = quitar compensacion de muneca");
  Serial.println("          o7 = otro lado actual de la muneca | a7 90 170 10 = angulos muneca");
  Serial.println("          s = ver calibracion | t3 = probar servo 3 | a3 10 170 = angulos servo 3 | r = contadores");
  Serial.println("          h = ir a HOME y quedarse (h otra vez = seguir al guante; tambien rearma tras falla de tension)");
  Serial.println("          v = ver tension de los servos | v 5.12 = corregir con lo que marca el tester");

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
  if (calEstado == CAL_MUNECA) avanzarCalibracionMuneca();
  else if (calEstado != CAL_NO) avanzarCalibracionGuiada();

  // Sin paquetes: los servos se quedan en la ultima posicion (o vuelven a HOME)
  if (tUltimoRx && !sinSenalAvisado && millis() - tUltimoRx > TIMEOUT_SENAL_MS) {
    Serial.println(VOLVER_HOME_SIN_SENAL ? ">>> SIN SENAL del transmisor"
                                         : ">>> SIN SENAL del transmisor (servos quietos en la ultima posicion)");
    sinSenalAvisado = true;
    if (VOLVER_HOME_SIN_SENAL) irAHome(false);
  }

  revisarTension();
  actualizarHome();
  actualizarServos();

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
