// =====================================================
// RECEPTOR - ESP32 (mano robot, COM3)
// Recibe por ESP-NOW los 8 flex y los 10 potenciometros del guante y mueve
// un servo por sensor (PCA9685) segun la calibracion (estirado -> doblado).
// Cada sensor con su servo es un "canal" (1 a 16, ver SERVOS[]):
//
//   canal 1-5   flex 1 al 5               -> MG90S  S1 a S5   (PCA 0 a 4)
//   canal 6-7   flex 6 y 7 (muneca)       -> DS3225 S11 y S12 (PCA 10 y 11)
//               flex 8 (costado del pulgar de la muneca) no tiene servo ni
//               canal propio: se usa para la compensacion de muneca
//   canal 8-11  pot. base (apertura)      -> MG92B  S13 a S16 (PCA 12 a 15)
//               pulgar, indice, anular, menique
//   canal 12-15 pot. arriba (nudillo)     -> MG90S  S6, S7, S9, S10 (PCA 5, 6, 8, 9)
//               pulgar, indice, anular, menique
//   canal 16    pot. arriba medio (M2 C3) -> MG90S  S8 (PCA 7)
//   El dedo medio no tiene apertura lateral: su potenciometro de la base no se usa.
//   (segun Conexionados/Circuito_mano_robot)
//
// MUNECA: flex 6 (adelante), flex 7 y flex 8 (los dos costados). Los tres se
// arrastran entre si por como estan en el guante, asi que con la muneca
// calibrada (comando m) se mueven juntos (ver COMPENSACION DE LA MUNECA):
//   - el costado sale SOLO de los flex 7 y 8: si cambian en la misma proporcion
//     que al ir hacia adelante (los dos casi igual) no es costado; si uno sube
//     y el otro baja (o no cambia), es costado.
//   - adelante sale del flex 6, restandole lo que le sube por el costado.
// El servo lateral (S12) es uno solo y va hacia los dos lados:
//   muneca al menique -> anguloDoblado (170) | derecha -> anguloPlano (90) | al pulgar -> anguloOtroLado (10)
// Si el servo gira al reves, intercambiar esos dos angulos (a7 90 10 170).
//
// Sin la calibracion de muneca (o despues de x), cada flex mueve su servo como
// antes: flex 6 adelante y flex 7 PRE-CURVADO hacia los dos lados (sube R ->
// anguloDoblado, baja R -> anguloOtroLado); el flex 8 no se usa.
//
// La calibracion se guarda en la memoria del ESP32 (no se pierde al
// apagar). Al arrancar, el estirado se vuelve a medir con los primeros
// paquetes: arrancar con la mano del guante ESTIRADA, los dedos JUNTOS
// y la muneca DERECHA (centro).
//
// Comandos por el monitor serie (escribir y Enter):
//   c      : calibracion guiada en 3 fases: mano abierta con dedos juntos (4 s),
//            cerrar la mano y mover la muneca (10 s), separar los dedos (6 s)
//   p / d  : tomar el valor actual de TODOS los canales como estirado / doblado
//   p3, d3 : lo mismo pero solo para el canal 3 (1 a 16)
//   m      : calibracion de muneca con el guante puesto: centro, adelante, costado
//            del menique y del pulgar (7 s cada postura). Mide cuanto cambian los
//            flex 6, 7 y 8 en cada postura y separa los dos movimientos.
//            Con la muneca compensada, c solo vuelve a tomar su centro.
//   x      : quitar la compensacion de muneca (cada flex por separado)
//   o7     : tomar la resistencia actual del flex 7 como "otro lado"
//            (solo flex bidireccionales)
//   s      : mostrar la calibracion
//   t3     : prueba del servo del canal 3 (barrido estirado -> doblado -> estirado;
//            si es bidireccional sigue -> otro lado -> centro)
//   a3 10 170 : angulos del servo del canal 3 (estirado, doblado). Es temporal, para
//               probar: los definitivos se escriben en SERVOS[] mas abajo.
//   a7 90 170 10 : angulos del servo 7 (centro, un lado, otro lado)
//   h      : llevar la mano a HOME y dejarla quieta ahi (h otra vez: sigue al guante)
//   v      : ver la tension de los servos; v 5.12 = corregirla con lo que marca el tester
//   w      : modo ajuste: muestra solo los potenciometros (para acomodarlos), w para salir
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
// Servos de los dedos (flex S1-S5 y nudillos S6-S10): pulso ampliado para sacarles mas recorrido. El
// servo gira mientras su potenciometro interno lo permita (~190-210 grados);
// si en un extremo zumba o se descontrola, achicar estos valores.
#define MG90S_DEDOS_MIN_US 400
#define MG90S_DEDOS_MAX_US 2600
#define MG92B_MIN_US  500
#define MG92B_MAX_US  2500

// De donde sale el valor que mueve cada servo
enum { SENSOR_FLEX, SENSOR_POT };

typedef struct {
  const char *nombre;
  const char *etiqueta;     // nombre corto del sensor (columnas del historial)
  uint8_t  tipoSensor;      // SENSOR_FLEX (resistencia en ohm) o SENSOR_POT (cuentas del ADS)
  uint8_t  sensor;          // indice en resistencia[] o pot[] del paquete
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

// Un "canal" = un sensor del guante que mueve un servo de la mano.
// Los comandos usan el numero de canal (1 a 16): t3, a12 10 100, p9...
//   1-7  : flex 1 a 7
//   8-11 : potenciometros de la BASE (apertura lateral) -> MG92B S13 a S16
//   12-15: potenciometros de ARRIBA (flexion del nudillo) -> MG90S S6, S7, S9, S10
//   16   : potenciometro de ARRIBA del dedo medio -> MG90S S8 (canal 7 del PCA9685)
// El dedo medio no tiene apertura lateral (casi no se mueve): su potenciometro
// de la base no se usa.
//
// Apertura (MG92B, canales 8-11): el dedo se mueve poco pero el servo usa
// 180 grados (0 -> 180), porque la calibracion estira el movimiento a todo el rango.
// (el horn de cada servo va colocado con los dedos JUNTOS a 0 grados)
// Los angulos de los servos NUEVOS (canales 8 a 16) son provisorios y
// conservadores: probarlos con t8 ... t16 y ajustarlos con a8 ... a16.
#define CANT_CANALES 16
//                          sensor                    PCA  pulso                       estirado doblado otro lado   HOME
ServoDedo SERVOS[CANT_CANALES] = {
  {"S1 MG90S",   "F1",   SENSOR_FLEX, 0,                0, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 1 flex 1
  {"S2 MG90S",   "F2",   SENSOR_FLEX, 1,                1, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 2 flex 2
  {"S3 MG90S",   "F3",   SENSOR_FLEX, 2,                2, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 3 flex 3
  {"S4 MG90S",   "F4",   SENSOR_FLEX, 3,                3, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 4 flex 4
  {"S5 MG90S",   "F5",   SENSOR_FLEX, 4,                4, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 5 flex 5
  {"S11 DS3225", "F6",   SENSOR_FLEX, 5,               10, DS3225_MIN_US, DS3225_MAX_US, 10, 170, SIN_OTRO_LADO,  10},  // 6 flex 6 (muneca adelante)
  {"S12 DS3225", "F7",   SENSOR_FLEX, 6,               11, DS3225_MIN_US, DS3225_MAX_US, 90, 170, 10,             90},  // 7 flex 7 (muneca lateral)
  {"S13 MG92B",  "Bpul", SENSOR_POT,  POT_BASE + 0,    12, MG92B_MIN_US,  MG92B_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 8 apertura pulgar
  {"S14 MG92B",  "Bind", SENSOR_POT,  POT_BASE + 1,    13, MG92B_MIN_US,  MG92B_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 9 apertura indice
  {"S15 MG92B",  "Banu", SENSOR_POT,  POT_BASE + 3,    14, MG92B_MIN_US,  MG92B_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 10 apertura anular
  {"S16 MG92B",  "Bmen", SENSOR_POT,  POT_BASE + 4,    15, MG92B_MIN_US,  MG92B_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 11 apertura menique
  {"S6 MG90S",   "Apul", SENSOR_POT,  POT_ARRIBA + 0,   5, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 12 nudillo pulgar
  {"S7 MG90S",   "Aind", SENSOR_POT,  POT_ARRIBA + 1,   6, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 13 nudillo indice
  {"S9 MG90S",   "Aanu", SENSOR_POT,  POT_ARRIBA + 3,   8, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 14 nudillo anular
  {"S10 MG90S",  "Amen", SENSOR_POT,  POT_ARRIBA + 4,   9, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 15 nudillo menique
  {"S8 MG90S",   "Amed", SENSOR_POT,  POT_ARRIBA + 2,   7, MG90S_DEDOS_MIN_US, MG90S_DEDOS_MAX_US,   0, 180, SIN_OTRO_LADO,   0},  // 16 nudillo medio (M2 C3)
};

// Valor del sensor de cada canal en un paquete (ohm para flex, cuentas para potenciometros)
float valorCanal(const PaqueteGuante &p, int i) {
  const ServoDedo &s = SERVOS[i];
  return s.tipoSensor == SENSOR_FLEX ? (float)p.resistencia[s.sensor] : (float)p.pot[s.sensor];
}
bool esPotBase(int i) { return SERVOS[i].tipoSensor == SENSOR_POT && SERVOS[i].sensor < POT_ARRIBA; }

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
#define ALPHA 0.3f

// Histeresis: el conversor del guante mide de a escalones de ~2 mV y una
// lectura que queda justo entre dos escalones salta de uno a otro. Con un flex
// de poco recorrido (~200 ohm entre estirado y doblado) un escalon son 20-30
// grados y el dedo tiembla quieto. El valor que mueve al servo solo cambia
// cuando el filtrado se aleja mas de HISTERESIS_ESCALONES escalones.
#define HISTERESIS_ESCALONES 0.8f
#define ESCALON_ADS_V 0.002f      // 16 cuentas del ADS a +-4.096 V (ver transmisor)
#define ESCALON_POT   16.0f       // el mismo escalon, en cuentas
#define R_FIJA_GUANTE 10000.0f    // divisor de cada flex en el guante
#define VCC_GUANTE    3.3f

// Cuantos ohm cambia un flex de resistencia r con un escalon del conversor
float escalonFlex(float r) {
  return (R_FIJA_GUANTE + r) * (R_FIJA_GUANTE + r) / (R_FIJA_GUANTE * VCC_GUANTE) * ESCALON_ADS_V;
}

// Sigue al valor medido, pero solo cuando se aleja mas que la banda
float aplicarHisteresis(float anterior, float medido, float banda) {
  if (anterior < 0) return medido;
  if (medido > anterior + banda) return medido - banda;
  if (medido < anterior - banda) return medido + banda;
  return anterior;
}

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

// Calibracion guiada (comando c), en 3 fases:
//   1 ABIERTA : mano abierta, dedos estirados y JUNTOS, muneca derecha -> estirado de todo
//   2 CERRADA : cerrar la mano (flex y nudillos) y mover la muneca      -> doblado de flex y pot. de arriba
//   3 SEPARAR : mano abierta, separar los dedos hacia los costados      -> doblado de pot. de la base
#define CAL_ABIERTA_MS 4000
#define CAL_CERRADA_MS 10000   // tiempo para cerrar la mano y llevar la muneca a todos lados
#define CAL_SEPARAR_MS 6000    // tiempo para abrir y separar los dedos
#define RANGO_MINIMO   150     // ohms: si un flex cambia menos, no se calibra
                               // (con el guante puesto el pulgar y el anular cambian ~250)
#define RANGO_MINIMO_POT 300   // cuentas (~37 mV, ~3 grados del potenciometro)
// Limites de lectura del potenciometro (modo ajuste, comando w)
#define POT_TOPE       26300   // ~3.3 V: el potenciometro esta al final de su giro
#define POT_CERCA_TOPE 23000   // con el dedo estirado no deberia pasar de aca
#define POT_PISO       200     // ~0 V: el otro final del giro
#define MARGEN_DOBLADO 0.05f   // el doblado queda un 5% antes del maximo medido

// Potenciometros sin calibrar: estirado = doblado (el servo queda quieto en
// el angulo de estirado hasta que se calibren con c). Su sentido (sube o baja
// al mover el dedo) depende de como esta montado: la calibracion lo detecta.

// Flex bidireccional (muneca lateral, pre-curvado): hacia el pulgar el flex
// se endereza y BAJA la resistencia
#define RANGO_INICIAL_OTRO_LADO 500   // ohms por debajo del centro si nunca se calibro
#define RANGO_MINIMO_OTRO_LADO  100   // ohms: si cambia menos, no se calibra ese lado
// El flex no vuelve siempre exactamente al mismo valor en el centro: zona
// muerta alrededor del centro donde el servo queda quieto en el centro
#define ZONA_CENTRO 0.10f             // 10% del recorrido hacia el menique
#define ZONA_CENTRO_OTRO_LADO 0.10f   // 10% del recorrido hacia el pulgar (pre-curvado:
                                      // los dos lados tienen recorrido parecido)

// =====================================================
// COMPENSACION DE LA MUNECA (flex 6, 7 y 8)
// =====================================================
// Por como estan en el guante, los flex de la muneca se arrastran entre si:
// al doblarla hacia adelante suben el 6 y TAMBIEN los dos laterales (7 y 8),
// y al llevarla a un costado el 6 tambien cambia un poco. El flex 8 (costado
// del pulgar) se agrego para poder separar los dos movimientos.
// La calibracion de muneca (comando m) mide cuanto cambia cada flex en cada
// postura (adelante, costado del menique, costado del pulgar) con el guante
// puesto, y en cada paquete:
//   1) COSTADO, solo con los flex 7 y 8: hacia adelante cambian en una
//      proporcion fija (casi iguales); hacia un costado uno sube y el otro baja
//      o no cambia. Lo que (R7, R8) se aparta de la proporcion de adelante es
//      costado; lo que la respeta es adelante y NO mueve el servo lateral.
//      (si adelante los dos subieran exactamente igual, el costado seria
//      simplemente la diferencia R8 - R7). El sentido de ese apartamiento dice
//      hacia que costado va, y su tamano comparado con el de la calibracion,
//      cuanto (0 a 1).
//   2) ADELANTE, con el flex 6, restandole lo que sube por ese costado
//      (medido en la calibracion): un costado puro NO mueve el servo de adelante.
#define FLEX_MUNECA_ADELANTE 5   // flex 6 (indice 5): adelante, servo S11 (canal 6)
#define FLEX_MUNECA_LATERAL  6   // flex 7 (indice 6): costado, servo S12 (canal 7)
#define FLEX_MUNECA_PULGAR   7   // flex 8 (indice 7): costado del pulgar, sin servo propio
// Los tres flex de la muneca, en el orden de los arreglos de la compensacion
const uint8_t FLEX_MUNECA[3] = {FLEX_MUNECA_ADELANTE, FLEX_MUNECA_LATERAL, FLEX_MUNECA_PULGAR};
enum { M6, M7, M8 };

// Calibracion de muneca: un costado se usa si los flex 7 y 8 cambian al menos
// esto (entre los dos) y distinto que hacia adelante: SEPARACION_MINIMA = 0
// seria igual que adelante, 1 = totalmente distinto (por ejemplo, adelante los
// dos suben igual y hacia el costado uno sube y el otro baja lo mismo)
#define RANGO_MINIMO_COSTADO 150      // ohms
#define SEPARACION_MINIMA    0.30f
// Si hacia adelante los flex 7 y 8 casi no cambian, se toma que adelante
// los mueve igual (cualquier cambio igual en los dos no es costado)

// El flex 6 tampoco vuelve siempre al mismo valor (+~90 ohm despues de doblarlo)
#define ZONA_CENTRO_ADELANTE 0.20f
// Zona muerta del costado (fraccion del recorrido de la calibracion). Despues
// de doblarse, el flex 7 tarda unos segundos en volver a su valor, y cerrar el
// puno tira de los flex 7 y 8: con la muneca derecha eso da hasta ~40% de
// costado (prueba del 2026-10-31). Mas grande = menos falsos movimientos pero
// el servo lateral arranca mas tarde.
#define ZONA_CENTRO_COSTADO 0.30f
// Cuanto suben los laterales al doblar hacia adelante varia un poco, y la
// diferencia parece un costado: con la muneca hacia adelante la zona muerta
// del costado crece hasta este porcentaje del recorrido que queda
#define ZONA_LATERAL_EXTRA_ADELANTE 0.40f

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

uint32_t rPlano[CANT_CANALES], rDoblado[CANT_CANALES];
uint32_t rOtroLado[CANT_CANALES];   // solo flex bidireccionales: resistencia doblado hacia el otro lado
bool calibracionGuardada = false;
int canalesGuardados = 0;          // canales con calibracion en la memoria (una vieja puede tener menos)
bool modoAjuste = false;           // comando w: muestra solo los potenciometros
float rFiltrada[CANT_CANALES];
float rFiltradaMuneca[3];            // R6, R7, R8 filtradas (el flex 8 no es un canal)
float rEstable[CANT_CANALES];        // filtradas + histeresis: las que mueven los servos
float rEstableMuneca[3];
float anguloServo[CANT_CANALES];     // angulo que tiene cada servo ahora
float objetivoServo[CANT_CANALES];   // angulo al que tiene que ir (segun el guante)
bool hayObjetivo = false;
uint32_t ultimaR[CANT_CANALES];
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
uint64_t calSuma[CANT_CANALES];
uint64_t calSumaMuneca[3];   // R6, R7, R8 (para volver a tomar el centro de la muneca compensada)

// Calibracion guiada
enum { CAL_NO, CAL_ABIERTA, CAL_CERRADA, CAL_SEPARAR, CAL_MUNECA } calEstado = CAL_NO;
uint32_t calInicio = 0, calUltimoAviso = 0;
float calMax[CANT_CANALES], calMin[CANT_CANALES];

// Calibracion de muneca (comando m): una fase por postura
enum { POSTURA_CENTRO, POSTURA_ADELANTE, POSTURA_MENIQUE, POSTURA_PULGAR, CANT_POSTURAS };

// Compensacion de la muneca: valores de los flex 6, 7 y 8 (en el orden M6, M7, M8)
typedef struct {
  float centro[3];                  // resistencia con la muneca derecha (guante puesto)
  float cambio[CANT_POSTURAS][3];   // cuanto cambia cada flex en cada postura respecto
                                    // del centro (la fila POSTURA_CENTRO no se usa)
} CalMuneca;
CalMuneca muneca;
bool munecaCalibrada = false;
typedef struct {
  uint8_t postura;
  uint16_t duracionMs;
  const char *aviso;    // lo que se imprime (y lee en voz alta el script de calibracion)
} FaseMuneca;
const FaseMuneca FASES_MUNECA[] = {
  {POSTURA_CENTRO,   CAL_CENTRO_MS,  "Muneca derecha y quieta"},
  {POSTURA_ADELANTE, CAL_POSTURA_MS, "Muneca hacia adelante, al maximo, y mantener"},
  {POSTURA_CENTRO,   CAL_CENTRO_MS,  "Volver al centro"},
  {POSTURA_MENIQUE,  CAL_POSTURA_MS, "Muneca al costado del menique, al maximo, y mantener"},
  {POSTURA_CENTRO,   CAL_CENTRO_MS,  "Volver al centro"},
  {POSTURA_PULGAR,   CAL_POSTURA_MS, "Muneca al costado del pulgar, al maximo, y mantener"},
  {POSTURA_CENTRO,   CAL_CENTRO_MS,  "Volver al centro"},
};
const int CANT_FASES_MUNECA = sizeof(FASES_MUNECA) / sizeof(FASES_MUNECA[0]);
int faseMuneca = 0;
float munSuma[CANT_POSTURAS][3];
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
  for (int i = 0; i < CANT_CANALES; i++) {
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
  for (int i = 0; i < CANT_CANALES; i++) pca.setPWM(SERVOS[i].canal, 0, 4096);   // sin pulso: servo suelto
}

// Vuelve a encender los servos llevandolos a HOME de a uno (no se sabe donde
// quedaron mientras estuvieron sueltos)
void encenderServosEnHome() {
  servosApagados = false;
  for (int i = 0; i < CANT_CANALES; i++) {
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
  static bool moviendo[CANT_CANALES];
  uint32_t ahora = millis();
  if (ahora - tAnterior < PERIODO_SERVOS_MS) return;
  float dt = min(ahora - tAnterior, (uint32_t)50) / 1000.0f;
  tAnterior = ahora;
  if (enHome || !hayObjetivo) {               // en HOME manda actualizarHome()
    for (int i = 0; i < CANT_CANALES; i++) moviendo[i] = false;
    return;
  }

  float pasoMax = VELOCIDAD_MAXIMA * dt;
  for (int i = 0; i < CANT_CANALES; i++) {
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

// =====================================================
// COMPENSACION DE LA MUNECA (ver la explicacion arriba)
// =====================================================
// Proporcion en que cambian los flex 7 y 8 al ir hacia adelante (como vector
// de largo 1). Si adelante casi no los mueve: "los dos iguales".
void proporcionAdelante(const CalMuneca &c, float &p7, float &p8) {
  p7 = c.cambio[POSTURA_ADELANTE][M7];
  p8 = c.cambio[POSTURA_ADELANTE][M8];
  float largo = hypotf(p7, p8);
  if (largo < RANGO_MINIMO_COSTADO) { p7 = p8 = 1.0f; largo = sqrtf(2.0f); }
  p7 /= largo;
  p8 /= largo;
}

// Cuanto se aparta un cambio (x7, x8) de los flex laterales de la proporcion
// de adelante (en ohm). 0 = adelante puro o centro; el signo dice el costado.
float desvioCostado(const CalMuneca &c, float x7, float x8) {
  float p7, p8;
  proporcionAdelante(c, p7, p8);
  return p7 * x8 - p8 * x7;
}

// Un costado se usa si en la calibracion los flex 7 y 8 cambiaron bastante y
// distinto que hacia adelante (desvio / cambio = 0 igual que adelante, 1 opuesto)
bool costadoValido(const CalMuneca &c, int postura) {
  float l7 = c.cambio[postura][M7], l8 = c.cambio[postura][M8];
  float largo = hypotf(l7, l8);
  return largo >= RANGO_MINIMO_COSTADO && fabsf(desvioCostado(c, l7, l8)) >= SEPARACION_MINIMA * largo;
}

// Postura de la muneca a partir del cambio de R6, R7 y R8 respecto del centro:
//   f = hacia adelante (0 = derecha, 1 = como en la calibracion)
//   s = costado (+ menique, - pulgar; 1 = como en la calibracion)
void posturaMuneca(const float x[3], float &f, float &s) {
  // 1) Costado, solo con los flex 7 y 8: el desvio respecto de la proporcion
  //    de adelante se compara con el de cada costado en la calibracion
  float d = desvioCostado(muneca, x[M7], x[M8]);
  float arrastre6 = 0;   // lo que cambia el flex 6 por el costado
  s = 0;
  const int lados[] = {POSTURA_MENIQUE, POSTURA_PULGAR};
  for (int lado : lados) {
    float dLado = desvioCostado(muneca, muneca.cambio[lado][M7], muneca.cambio[lado][M8]);
    if (!costadoValido(muneca, lado) || d * dLado <= 0) continue;   // es hacia el otro costado
    float t = d / dLado;
    // Solo se resta lo que el costado SUBE al flex 6: si lo baja, no se suma
    // (no siempre baja lo mismo, y sumarlo de mas parecia muneca adelante;
    // probado 2026-10-31: hacia el pulgar bajo 180 ohm en la calibracion y
    // nada en la prueba). Que baje solo lleva el servo de adelante a derecha.
    arrastre6 = max(0.0f, t * muneca.cambio[lado][M6]);
    s = lado == POSTURA_MENIQUE ? t : -t;
    break;
  }
  // 2) Adelante, con el flex 6 sin lo que le sube por el costado
  f = (x[M6] - arrastre6) / muneca.cambio[POSTURA_ADELANTE][M6];
}

// Muneca compensada: angulos del servo de adelante y del lateral a partir de
// R6, R7 y R8 juntos
void calcularMuneca(const float r[3], float &angulo6, float &angulo7) {
  float x[3], f, s;
  for (int k = 0; k < 3; k++) x[k] = r[k] - muneca.centro[k];
  posturaMuneca(x, f, s);

  // Si R6 baja (por ejemplo, al cerrar el puno el guante tira del flex) f da
  // negativo: se toma como muneca derecha
  const ServoDedo &sa = SERVOS[FLEX_MUNECA_ADELANTE], &sl = SERVOS[FLEX_MUNECA_LATERAL];
  f = constrain(f, 0.0f, 1.0f);
  angulo6 = sa.anguloPlano + aplicarZonaCentro(f, ZONA_CENTRO_ADELANTE) * (sa.anguloDoblado - sa.anguloPlano);
  // Muneca hacia adelante: costado menos sensible
  float zona = ZONA_CENTRO_COSTADO + (1.0f - ZONA_CENTRO_COSTADO) * ZONA_LATERAL_EXTRA_ADELANTE * f;
  float t = aplicarZonaCentro(constrain(fabsf(s), 0.0f, 1.0f), zona);
  angulo7 = sl.anguloPlano + t * ((s >= 0 ? sl.anguloDoblado : sl.anguloOtroLado) - sl.anguloPlano);
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
    Serial.printf(">>> Prueba %s (canal %d, PCA %d): %d -> %d -> %d -> %d -> %d grados\n",
                  s.nombre, i + 1, s.canal, s.anguloPlano, s.anguloDoblado, s.anguloPlano,
                  s.anguloOtroLado, s.anguloPlano);
  else
    Serial.printf(">>> Prueba %s (canal %d, PCA %d): %d -> %d -> %d grados\n",
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
  canalesGuardados = CANT_CANALES;
}

// La calibracion de muneca se guarda aparte (la de antes, de 2 flex, estaba
// en "acople" y ya no se usa)
void guardarMuneca() {
  memoria.putBytes("muneca", &muneca, sizeof(muneca));
  memoria.putBool("munecaOk", munecaCalibrada);
}

// Canales de la muneca con la compensacion calibrada (se manejan juntos)
bool esMunecaCompensada(int i) {
  return munecaCalibrada && (i == FLEX_MUNECA_ADELANTE || i == FLEX_MUNECA_LATERAL);
}

// Vuelve a tomar el centro de la muneca compensada (los flex no vuelven
// siempre al mismo valor; cuanto cambian en cada postura si se mantiene).
// Si la muneca parece doblada, deja el centro guardado.
bool recentrarMuneca(const float medido[3]) {
  float x[3], f, s;
  for (int k = 0; k < 3; k++) x[k] = medido[k] - muneca.centro[k];
  posturaMuneca(x, f, s);
  if (f > 0.5f || fabsf(s) > 0.5f) {
    Serial.printf(">>> La muneca parece doblada (adelante %.0f%%, costado %+.0f%%): se deja el centro guardado\n",
                  100 * f, 100 * s);
    return false;
  }
  for (int k = 0; k < 3; k++) muneca.centro[k] = medido[k];
  return true;
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

// Lee un arreglo de la memoria aunque tenga menos canales que ahora (una
// calibracion vieja de 7 flex sirve igual: los canales nuevos quedan sin
// calibrar). Devuelve cuantos canales se leyeron.
int cargarArreglo(const char *clave, uint32_t *arreglo) {
  size_t largo = memoria.isKey(clave) ? memoria.getBytesLength(clave) : 0;
  if (largo == 0 || largo % sizeof(uint32_t) || largo > CANT_CANALES * sizeof(uint32_t)) return 0;
  memoria.getBytes(clave, arreglo, largo);
  return largo / sizeof(uint32_t);
}

bool canalCalibrado(int i) { return calibracionGuardada && i < canalesGuardados; }

void cargarCalibracion() {
  for (int i = 0; i < CANT_CANALES; i++) {
    rPlano[i] = R_PLANO_INICIAL;
    rDoblado[i] = SERVOS[i].tipoSensor == SENSOR_FLEX ? R_PLANO_INICIAL + RANGO_INICIAL : R_PLANO_INICIAL;
    rOtroLado[i] = otroLadoInicial(R_PLANO_INICIAL);
  }
  canalesGuardados = min(cargarArreglo("rPlano", rPlano), cargarArreglo("rDoblado", rDoblado));
  bool ok = canalesGuardados > 0;
  calibracionGuardada = ok;
  if (ok) {
    // Los canales que no estaban guardados vuelven al valor por defecto
    for (int i = canalesGuardados; i < CANT_CANALES; i++) {
      rPlano[i] = R_PLANO_INICIAL;
      rDoblado[i] = SERVOS[i].tipoSensor == SENSOR_FLEX ? R_PLANO_INICIAL + RANGO_INICIAL : R_PLANO_INICIAL;
    }
    // Calibraciones viejas no tienen el otro lado: se usa el valor por defecto
    cargarArreglo("rOtroLado", rOtroLado);
    for (int i = 0; i < CANT_CANALES; i++)
      if (!otroLadoValido(i)) rOtroLado[i] = otroLadoInicial(rPlano[i]);
    if (canalesGuardados < CANT_CANALES)
      Serial.printf("Calibracion guardada de %d canales: los canales %d a %d estan SIN CALIBRAR (hacer c)\n",
                    canalesGuardados, canalesGuardados + 1, CANT_CANALES);
  }
  Serial.println(ok ? "Calibracion cargada de la memoria." : "Sin calibracion guardada: valores por defecto.");

  if (memoria.getBool("munecaOk", false) && memoria.isKey("muneca") &&
      memoria.getBytesLength("muneca") == sizeof(muneca)) {
    memoria.getBytes("muneca", &muneca, sizeof(muneca));
    munecaCalibrada = true;
  } else {
    Serial.println("Muneca sin compensar (flex 6, 7 y 8): hacer la calibracion de muneca (m) con el guante puesto.");
  }
}

void imprimirMuneca() {
  if (!munecaCalibrada) {
    Serial.println(">>> Muneca SIN compensar: cada flex mueve su servo (flex 7 pre-curvado: menique / centro / pulgar;"
                   " el flex 8 no se usa). Calibrar con m.");
    return;
  }
  const ServoDedo &sl = SERVOS[FLEX_MUNECA_LATERAL];
  Serial.println(">>> Muneca compensada (comando m), en ohm:        R6      R7      R8");
  Serial.printf(">>>   centro                                  %6.0f  %6.0f  %6.0f\n",
                muneca.centro[M6], muneca.centro[M7], muneca.centro[M8]);
  const char *nombres[] = {"", "adelante", "costado menique", "costado pulgar"};
  for (int k = POSTURA_ADELANTE; k < CANT_POSTURAS; k++) {
    Serial.printf(">>>   %-16s (cambio)               %+6.0f  %+6.0f  %+6.0f", nombres[k],
                  muneca.cambio[k][M6], muneca.cambio[k][M7], muneca.cambio[k][M8]);
    if (k == POSTURA_ADELANTE) Serial.printf("   -> S11 a %d gr", SERVOS[FLEX_MUNECA_ADELANTE].anguloDoblado);
    else if (!costadoValido(muneca, k)) Serial.print("   -> NO SE USA (no se distingue de adelante)");
    else Serial.printf("   -> S12 a %d gr", k == POSTURA_MENIQUE ? sl.anguloDoblado : sl.anguloOtroLado);
    Serial.println();
  }
}

void imprimirCalibracion() {
  Serial.println(">>> Calibracion (flex en ohm, potenciometros en cuentas del ADS):");
  Serial.println(">>>   canal  sensor  servo       PCA   estirado -> gr    doblado -> gr    otro lado -> gr");
  for (int i = 0; i < CANT_CANALES; i++) {
    const ServoDedo &s = SERVOS[i];
    Serial.printf(">>>    %2d    %-5s   %-11s %2d    %6lu -> %3d     %6lu -> %3d",
                  i + 1, s.etiqueta, s.nombre, s.canal, (unsigned long)rPlano[i], s.anguloPlano,
                  (unsigned long)rDoblado[i], s.anguloDoblado);
    if (esBidireccional(i))
      Serial.printf("     %6lu -> %3d", (unsigned long)rOtroLado[i], s.anguloOtroLado);
    if (esMunecaCompensada(i)) Serial.print("     (no se usa: muneca compensada, ver abajo)");
    else if (rPlano[i] == rDoblado[i]) Serial.print("     (SIN CALIBRAR: servo quieto)");
    Serial.println();
  }
  imprimirMuneca();
}

void imprimirEncabezado() {
  Serial.print("RECIBIDO    tiempo     seq |");
  for (int i = 0; i < CANT_CANALES; i++) {
    Serial.printf(" %5s", SERVOS[i].etiqueta);
    if (i == FLEX_MUNECA_LATERAL) Serial.print("    F8");   // flex 8: sin servo, despues del 7
  }
  Serial.print(" |");
  for (int i = 0; i < CANT_CANALES; i++) Serial.printf(" A%-2d", i + 1);
  Serial.println(" (gr) | perdidos");
}

// Promedio de R6, R7 y R8 juntado en calSumaMuneca
void promedioMuneca(float medido[3]) {
  for (int k = 0; k < 3; k++) medido[k] = (float)calSumaMuneca[k] / calCant;
}

void terminarCalibracionInicio() {
  if (munecaCalibrada) {
    float medido[3];
    promedioMuneca(medido);
    recentrarMuneca(medido);
  }
  for (int i = 0; i < CANT_CANALES; i++) {
    if (esMunecaCompensada(i)) continue;   // ya se tomo el centro de la muneca
    uint32_t medido = calSuma[i] / calCant;
    // Nunca se calibro: estirado medido; flex = estirado + rango tipico,
    // potenciometro = sin movimiento hasta calibrarlo con c
    if (!canalCalibrado(i)) {
      rPlano[i] = medido;
      rDoblado[i] = SERVOS[i].tipoSensor == SENSOR_FLEX ? medido + RANGO_INICIAL : medido;
      rOtroLado[i] = otroLadoInicial(medido);
      continue;
    }
    // Si el dedo ya estaba doblado al arrancar, se deja el estirado guardado
    // (sirve para los dos sentidos: el potenciometro puede subir o bajar)
    float rango = (float)rDoblado[i] - (float)rPlano[i];
    if (rango != 0 && ((float)medido - (float)rPlano[i]) / rango > 0.5f) {
      Serial.printf(">>> Canal %d (%s) parece doblado al arrancar (%lu): se deja el estirado guardado\n",
                    i + 1, SERVOS[i].etiqueta, (unsigned long)medido);
      continue;
    }
    // Bidireccional: tambien puede estar doblado hacia el otro lado
    int32_t rangoOtro = (int32_t)rPlano[i] - (int32_t)rOtroLado[i];
    if (esBidireccional(i) && (int32_t)rPlano[i] - (int32_t)medido > rangoOtro / 2) {
      Serial.printf(">>> Canal %d (%s) parece doblado hacia el otro lado al arrancar (%lu): se deja el centro guardado\n",
                    i + 1, SERVOS[i].etiqueta, (unsigned long)medido);
      continue;
    }
    // Ya calibrado: el doblado y el otro lado se corren junto con el estirado
    // (el flex no vuelve siempre al mismo valor, pero cuanto cambia al doblarlo
    // si se mantiene)
    moverCentro(i, medido);
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
  for (int i = 0; i < CANT_CANALES; i++) calSuma[i] = 0;
  for (int k = 0; k < 3; k++) calSumaMuneca[k] = 0;
  // Las indicaciones "CALIBRACION n/3: ..." las lee en voz alta el script de calibracion
  Serial.println("\n>>> CALIBRACION 1/3: Mano abierta, dedos estirados y juntos, muneca derecha. Quieta.");
}

// Potenciometro: el sentido (sube o baja) depende de como esta montado, asi
// que el doblado es el extremo que mas se alejo del estirado
void calibrarPotenciometro(int i, float plano) {
  float sube = calMax[i] - plano, baja = plano - calMin[i];
  float rango = max(sube, baja);
  if (rango < RANGO_MINIMO_POT) {
    Serial.printf(">>> Canal %d (%s) casi no cambio (%.0f cuentas): se deja la calibracion anterior\n",
                  i + 1, SERVOS[i].etiqueta, rango);
    return;
  }
  rPlano[i] = (uint32_t)plano;
  rDoblado[i] = (uint32_t)(sube >= baja ? calMax[i] - MARGEN_DOBLADO * rango : calMin[i] + MARGEN_DOBLADO * rango);
  // Si se movio mucho para los dos lados puede que se haya movido otro dedo
  if (min(sube, baja) > 0.5f * rango)
    Serial.printf(">>> Canal %d (%s): cambio para los dos lados (+%.0f / -%.0f), revisar\n",
                  i + 1, SERVOS[i].etiqueta, sube, baja);
}

void avanzarCalibracionGuiada() {
  uint32_t ahora = millis();
  if (calEstado == CAL_ABIERTA && ahora - calInicio >= CAL_ABIERTA_MS) {
    if (calCant == 0) { Serial.println(">>> Sin datos: calibracion cancelada"); calEstado = CAL_NO; return; }
    for (int i = 0; i < CANT_CANALES; i++) calMax[i] = calMin[i] = calSuma[i] / calCant;
    calEstado = CAL_CERRADA;
    calInicio = calUltimoAviso = ahora;
    Serial.println(">>> CALIBRACION 2/3: Cerrar la mano fuerte, doblando los nudillos. Despues muneca adelante, al menique y al pulgar.");
  } else if (calEstado == CAL_CERRADA && ahora - calInicio >= CAL_CERRADA_MS) {
    calEstado = CAL_SEPARAR;
    calInicio = calUltimoAviso = ahora;
    Serial.println(">>> CALIBRACION 3/3: Abrir la mano y separar los dedos hacia los costados, al maximo.");
  } else if (calEstado == CAL_SEPARAR && ahora - calInicio >= CAL_SEPARAR_MS) {
    calEstado = CAL_NO;
    // Muneca compensada: de la fase 1 (muneca derecha) solo se toma el centro;
    // cuanto cambia en cada postura se calibra con m
    if (munecaCalibrada) {
      float medido[3];
      promedioMuneca(medido);
      if (recentrarMuneca(medido)) {
        guardarMuneca();
        Serial.println(">>> Muneca: se volvio a tomar el centro (lo demas se calibra con m)");
      }
    }
    for (int i = 0; i < CANT_CANALES; i++) {
      uint32_t plano = calSuma[i] / calCant;
      float rango = calMax[i] - plano;
      if (esMunecaCompensada(i)) continue;
      if (SERVOS[i].tipoSensor == SENSOR_POT) {
        calibrarPotenciometro(i, plano);
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
    uint32_t total = calEstado == CAL_ABIERTA ? CAL_ABIERTA_MS
                   : calEstado == CAL_CERRADA ? CAL_CERRADA_MS : CAL_SEPARAR_MS;
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
  for (int k = 0; k < CANT_POSTURAS; k++) {
    for (int j = 0; j < 3; j++) munSuma[k][j] = 0;
    munCant[k] = 0;
  }
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
  for (int j = 0; j < 3; j++) munSuma[k][j] += p.resistencia[FLEX_MUNECA[j]];
  munCant[k]++;
}

void terminarCalibracionMuneca() {
  calEstado = CAL_NO;
  for (int k = 0; k < CANT_POSTURAS; k++)
    if (munCant[k] == 0) { Serial.println(">>> Sin datos: calibracion de muneca cancelada"); return; }

  // Promedio de cada postura (el centro promedia todas las vueltas al centro:
  // el flex no vuelve siempre al mismo valor)
  CalMuneca nueva = {};
  for (int j = 0; j < 3; j++) nueva.centro[j] = munSuma[POSTURA_CENTRO][j] / munCant[POSTURA_CENTRO];
  for (int k = POSTURA_ADELANTE; k < CANT_POSTURAS; k++)
    for (int j = 0; j < 3; j++) nueva.cambio[k][j] = munSuma[k][j] / munCant[k] - nueva.centro[j];

  const char *nombres[] = {"centro", "adelante", "menique", "pulgar"};
  Serial.println(">>> Muneca, en ohm:                       R6      R7      R8");
  Serial.printf(">>>   centro                           %6.0f  %6.0f  %6.0f\n",
                nueva.centro[M6], nueva.centro[M7], nueva.centro[M8]);
  for (int k = POSTURA_ADELANTE; k < CANT_POSTURAS; k++)
    Serial.printf(">>>   %-10s (cambio)              %+6.0f  %+6.0f  %+6.0f\n", nombres[k],
                  nueva.cambio[k][M6], nueva.cambio[k][M7], nueva.cambio[k][M8]);

  // El servo llega al final un poco antes del maximo medido (igual que en la
  // calibracion guiada)
  for (int k = POSTURA_ADELANTE; k < CANT_POSTURAS; k++)
    for (int j = 0; j < 3; j++) nueva.cambio[k][j] *= 1.0f - MARGEN_DOBLADO;

  // Adelante tiene que mover bien el flex 6
  if (nueva.cambio[POSTURA_ADELANTE][M6] < 2 * RANGO_MINIMO) {
    Serial.printf(">>> El flex 6 casi no cambio hacia adelante (%+.0f ohm): calibracion cancelada\n",
                  nueva.cambio[POSTURA_ADELANTE][M6]);
    return;
  }

  // Cada costado tiene que mover los flex 7 y 8 distinto que adelante
  float p7, p8;
  proporcionAdelante(nueva, p7, p8);
  Serial.printf(">>> Hacia adelante los flex 7 y 8 cambian en proporcion %.2f : %.2f%s\n", p7, p8,
                hypotf(nueva.cambio[POSTURA_ADELANTE][M7], nueva.cambio[POSTURA_ADELANTE][M8]) < RANGO_MINIMO_COSTADO
                    ? " (casi no cambian: se toma 1 : 1)" : "");
  float desvio[CANT_POSTURAS];
  int validos = 0;
  for (int k = POSTURA_MENIQUE; k <= POSTURA_PULGAR; k++) {
    float l7 = nueva.cambio[k][M7], l8 = nueva.cambio[k][M8];
    float largo = hypotf(l7, l8);
    desvio[k] = desvioCostado(nueva, l7, l8);
    bool ok = costadoValido(nueva, k);
    validos += ok;
    Serial.printf(">>> Costado del %-8s: flex 7 y 8 cambian %4.0f ohm, distinto de adelante en %3.0f%% -> %s\n",
                  nombres[k], largo, largo > 0 ? 100 * fabsf(desvio[k]) / largo : 0.0f,
                  ok ? "OK" : largo < RANGO_MINIMO_COSTADO ? "NO SE USA (casi no cambian)"
                                                            : "NO SE USA (parece adelante)");
  }
  if (validos == 0) {
    Serial.println(">>> Ningun costado se distingue de adelante con los flex 7 y 8.");
    Serial.println(">>> Revisar la colocacion de los flex 7 y 8. Calibracion cancelada.");
    return;
  }
  if (validos == 2 && desvio[POSTURA_MENIQUE] * desvio[POSTURA_PULGAR] > 0) {
    Serial.println(">>> Los dos costados mueven los flex 7 y 8 hacia el mismo lado: no se pueden distinguir.");
    Serial.println(">>> Revisar la colocacion de los flex 7 y 8. Calibracion cancelada.");
    return;
  }
  if (validos == 1)
    Serial.println(">>> ATENCION: el servo lateral solo va a ir hacia un costado (el otro NO SE USA)");

  muneca = nueva;
  munecaCalibrada = true;
  guardarMuneca();
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

  float valor[CANT_CANALES];
  for (int i = 0; i < CANT_CANALES; i++) {
    valor[i] = valorCanal(p, i);
    ultimaR[i] = valor[i];
    rFiltrada[i] = rFiltrada[i] < 0 ? valor[i] : rFiltrada[i] + ALPHA * (valor[i] - rFiltrada[i]);
    float escalon = SERVOS[i].tipoSensor == SENSOR_FLEX ? escalonFlex(rFiltrada[i]) : ESCALON_POT;
    rEstable[i] = aplicarHisteresis(rEstable[i], rFiltrada[i], HISTERESIS_ESCALONES * escalon);
  }
  float rMuneca[3];
  for (int k = 0; k < 3; k++) {
    rMuneca[k] = p.resistencia[FLEX_MUNECA[k]];
    rFiltradaMuneca[k] = rFiltradaMuneca[k] < 0 ? rMuneca[k]
                                                : rFiltradaMuneca[k] + ALPHA * (rMuneca[k] - rFiltradaMuneca[k]);
    rEstableMuneca[k] = aplicarHisteresis(rEstableMuneca[k], rFiltradaMuneca[k],
                                          HISTERESIS_ESCALONES * escalonFlex(rFiltradaMuneca[k]));
  }

  // Calibracion del estirado con los primeros paquetes
  if (calibrandoInicio) {
    for (int i = 0; i < CANT_CANALES; i++) calSuma[i] += valor[i];
    for (int k = 0; k < 3; k++) calSumaMuneca[k] += rMuneca[k];
    if (++calCant >= PAQUETES_CALIBRACION) terminarCalibracionInicio();
    return;
  }

  // Calibracion guiada: junta datos (los servos siguen moviendose).
  // Cerrar la mano tambien mueve los potenciometros de la base, por eso la
  // apertura lateral se mide aparte (fase SEPARAR) y no mientras se cierra.
  if (calEstado == CAL_ABIERTA) {
    for (int i = 0; i < CANT_CANALES; i++) calSuma[i] += valor[i];
    for (int k = 0; k < 3; k++) calSumaMuneca[k] += rMuneca[k];
    calCant++;
  } else if (calEstado == CAL_CERRADA || calEstado == CAL_SEPARAR) {
    for (int i = 0; i < CANT_CANALES; i++) {
      if (esPotBase(i) != (calEstado == CAL_SEPARAR)) continue;
      calMax[i] = max(calMax[i], rFiltrada[i]);
      calMin[i] = min(calMin[i], rFiltrada[i]);
    }
  } else if (calEstado == CAL_MUNECA) {
    juntarDatosMuneca(p);
  }

  // Objetivo de cada servo (se calcula siempre, aun en HOME, para que al
  // salir de HOME el servo vaya directo a donde esta el guante)
  for (int i = 0; i < CANT_CANALES; i++) {
    if (esMunecaCompensada(i)) continue;
    fijarObjetivo(i, calcularAngulo(i, rEstable[i]));
  }
  if (munecaCalibrada) {
    float a6, a7;
    calcularMuneca(rEstableMuneca, a6, a7);
    fijarObjetivo(FLEX_MUNECA_ADELANTE, a6);
    fijarObjetivo(FLEX_MUNECA_LATERAL, a7);
  }

  // Modo ajuste (comando w): solo los potenciometros, para acomodarlos mirando
  // el numero. El potenciometro mide de 0 a ~26400 (3.3 V): con el dedo
  // ESTIRADO tiene que quedar lejos del tope para que al doblar no lo alcance.
  if (modoAjuste && p.seq % HISTORIAL_CADA_N == 0 && calEstado == CAL_NO) {
    Serial.print("AJUSTE |");
    for (int k = 0; k < CANT_POT; k++) {
      int16_t v = p.pot[k];
      const char *marca = v >= POT_TOPE ? "TOPE" : v >= POT_CERCA_TOPE ? "alto" : v <= POT_PISO ? "PISO" : "ok";
      Serial.printf(" %s%s %5d %-4s |", k < POT_ARRIBA ? "B" : "A",
                    (const char *[]){"pul", "ind", "med", "anu", "men"}[k % 5], v, marca);
    }
    Serial.println();
    return;
  }

  if (p.seq % HISTORIAL_CADA_N == 0 && calEstado == CAL_NO) {
    if (p.seq % (HISTORIAL_CADA_N * 40) == 0) imprimirEncabezado();
    Serial.printf("RECIBIDO %8.2f s  #%6lu |", r.t_rx_ms / 1000.0, (unsigned long)p.seq);
    for (int i = 0; i < CANT_CANALES; i++) {
      Serial.printf(" %5ld", lroundf(valor[i]));
      if (i == FLEX_MUNECA_LATERAL) Serial.printf(" %5ld", lroundf(rMuneca[M8]));   // flex 8
    }
    Serial.print(" |");
    for (int i = 0; i < CANT_CANALES; i++) Serial.printf(" %3d", (int)lroundf(anguloServo[i]));
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
  if (n < 0 || n > CANT_CANALES) { Serial.println(">>> Numero de canal invalido (1 a 16)"); return; }
  int desde = n ? n - 1 : 0, hasta = n ? n : CANT_CANALES;

  if (c == 'p' || c == 'd') {
    if (!hayDatos) { Serial.println(">>> Sin datos del guante"); return; }
    bool centrarMuneca = false;
    for (int i = desde; i < hasta; i++) {
      if (esMunecaCompensada(i)) {
        // Muneca compensada: p toma el centro de los 3 flex, el doblado se calibra con m
        if (c == 'p') centrarMuneca = true;
        else if (n) Serial.printf(">>> Flex %d (muneca): el doblado se calibra con el comando m\n", i + 1);
        continue;
      }
      (c == 'p' ? rPlano : rDoblado)[i] = (uint32_t)rFiltrada[i];
      if (esBidireccional(i) && !otroLadoValido(i)) rOtroLado[i] = otroLadoInicial(rPlano[i]);
    }
    if (centrarMuneca) {
      for (int k = 0; k < 3; k++) muneca.centro[k] = rFiltradaMuneca[k];
      guardarMuneca();
    }
    guardarCalibracion();
    imprimirCalibracion();
  } else if (c == 'o') {
    if (!hayDatos) { Serial.println(">>> Sin datos del guante"); return; }
    for (int i = desde; i < hasta; i++) {
      if (!esBidireccional(i)) {
        if (n) Serial.printf(">>> Canal %d no es bidireccional\n", i + 1);
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
    munecaCalibrada = false;   // vuelve a mover cada flex de la muneca por separado
    guardarMuneca();
    imprimirCalibracion();
  } else if (c == 's') {
    imprimirCalibracion();
    Serial.printf(">>> Recibidos %lu  perdidos %lu  largo distinto %lu\n", (unsigned long)recibidos,
                  (unsigned long)perdidos, (unsigned long)largoDistinto);
    comandoTension("");
    Wire.beginTransmission(0x40);
    Serial.println(Wire.endTransmission() == 0 ? ">>> PCA9685: responde por I2C (0x40)"
                                               : ">>> PCA9685: NO RESPONDE por I2C: revisar SDA (GPIO21), SCL (GPIO22), VCC y GND del PCA9685");
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
  } else if (c == 'w') {
    modoAjuste = !modoAjuste;
    Serial.println(modoAjuste ? ">>> MODO AJUSTE de potenciometros: con el dedo ESTIRADO dejar los de arriba (A...)"
                                " entre 16000 y 21000 y doblando que no lleguen a TOPE (w para salir)"
                              : ">>> Fin del modo ajuste");
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
  Serial.println("\n=== MANO ROBOT: 8 FLEX + 9 POTENCIOMETROS -> 16 SERVOS (ESP32 + PCA9685) ===");

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
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() != 0)
    Serial.println("ATENCION: el PCA9685 NO RESPONDE por I2C: revisar SDA (GPIO21), SCL (GPIO22), VCC y GND");
  pca.setPWMFreq(50);
  delay(10);

  // Los canales sin flex asignado quedan apagados (servo suelto)
  for (int canal = 0; canal < 16; canal++) {
    bool usado = false;
    for (int i = 0; i < CANT_CANALES; i++) usado |= SERVOS[i].canal == canal;
    if (!usado) pca.setPWM(canal, 0, 4096);   // 4096 = salida siempre apagada
  }

  // Lleva los servos a HOME de a uno (sin pico de corriente). El servo no
  // informa donde esta, asi que este primer movimiento es a su velocidad maxima;
  // los siguientes (h, sin senal) van despacio a VELOCIDAD_HOME
  for (int k = 0; k < 3; k++) rFiltradaMuneca[k] = rEstableMuneca[k] = -1;
  for (int i = 0; i < CANT_CANALES; i++) {
    rFiltrada[i] = rEstable[i] = -1;
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
  Serial.println("Comandos: c = calibracion guiada | p/d = estirado/doblado actual (p3 = solo canal 3, de 1 a 16)");
  Serial.println("          m = calibracion de muneca (flex 6, 7 y 8 juntos) | x = quitar compensacion de muneca");
  Serial.println("          o7 = otro lado actual de la muneca | a7 90 170 10 = angulos muneca");
  Serial.println("          s = ver calibracion | t3 = probar servo 3 | a3 10 170 = angulos servo 3 | r = contadores");
  Serial.println("          h = ir a HOME y quedarse (h otra vez = seguir al guante; tambien rearma tras falla de tension)");
  Serial.println("          v = ver tension de los servos | v 5.12 = corregir con lo que marca el tester");
  Serial.println("          w = modo ajuste de potenciometros (solo muestra los potenciometros)");

  cola = xQueueCreate(20, sizeof(Recepcion));
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecibido);

  if (calibrandoInicio)
    Serial.println("Calibrando: mantener la mano del guante ESTIRADA, los dedos JUNTOS y la muneca DERECHA hasta que aparezca la calibracion...");
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
