// =====================================================================
// PRUEBA DE DEDOS Y MUNECA - Mano robot (ESP32 del receptor + PCA9685), SIN guante
// =====================================================================
// Barre, de a un servo por vez, los rangos de pulgar, indice, medio y muneca.
// Mismo circuito que "5. Mano - ESP32(Receptor)": I2C en GPIO21 (SDA) / GPIO22 (SCL),
// PCA9685 en 0x40, mismos canales del PCA.
//
// Comandos por el monitor serie (115200):
//   h        -> ACTIVA los servos y los lleva despacio a HOME (dedos en su minimo = mano extendida,
//               muneca derecha: adelante/atras 90 y lateral 100 = los dos DS3225 en 100). Al encender los servos estan SUELTOS:
//               hay que mandar h primero.
//   1 / 2 / 3 / 4 -> prueba el pulgar / indice / medio / anular: barre cada servo del dedo min -> max -> min,
//               de a uno, con el resto en HOME. Pulgar: la apertura sale PRIMERO y se queda afuera
//               mientras se barren Flex 1 y el nudillo (si no, choca con la palma); vuelve al final.
//   0        -> prueba los 4 dedos, uno detras del otro
//   5        -> prueba la muneca: adelante/atras (centro -> adelante -> atras -> centro) y despues
//               lateral (centro -> menique -> pulgar -> centro)
//   w A L    -> muneca a adelante/atras A (70 atras .. 110 adelante) y lateral L (76 pulgar .. 120 menique)
//               a la vez. Ej: w 100 100 = un poco adelante, lateral derecho (en su cero)
//   d N      -> mueve el dedo N ENTERO (1 pulgar, 2 indice, 3 medio, 4 anular): todas sus articulaciones juntas
//               extendido -> flexionado -> extendido (llegan todas a la vez)
//   d N P    -> lleva el dedo N al P % (0 = extendido, 100 = flexionado) y lo deja ahi
//               (el lateral del indice no participa: se mueve aparte con m 6 A)
//   s N      -> barre solo el servo N (1 a 12, ver tabla con e; 9 y 10 = muneca, 11 y 12 = anular)
//   m N A    -> mueve el servo N al angulo A (se limita a su rango)
//   r N      -> rapidez (velocidad) en grados por segundo (por defecto 40)
//   e        -> estado: tabla de servos, angulos y rangos
//   ?        -> esta ayuda
//
// PARADA DE EMERGENCIA:
//   x (sola, sin Enter) frena todo en cualquier momento y deja los servos SUELTOS (sin pulso,
//   sin fuerza). Para seguir: h. Las otras teclas NO frenan: se juntan como un comando normal.
//
// COMPENSACION DEL PULGAR (al abrir la apertura la falange se cae: el nudillo la acompana):
//   El nudillo se mueve con la apertura como una articulacion acoplada: servo del nudillo =
//   lo que se le pide + K x (apertura - 41): hasta 41 (30 gr de apertura) NO compensa, para que el
//   pulgar salga de la palma antes de cerrar el nudillo. Lo que se le pide al nudillo (m 2 A, d 1...) es el
//   angulo "real" de la articulacion; la compensacion se suma por debajo.
//   c        -> activa / desactiva la compensacion (el cambio es suave, el nudillo no salta)
//   k        -> muestra K;  k 0.4 = cambiar K (grados de nudillo por grado de apertura)
//   g        -> CALIBRA K con una postura: apertura lejos (m 3 110), nudillo hasta que la falange
//               quede como un pulgar real EXTENDIDO (m 2 A), y g. Se guarda (sobrevive al apagado).
//   Para verla: s 3 (barre la apertura con la compensacion activa).
//
// MUNECA (diferencial con 2 DS3225: A = S11 en PCA 10, B = S12 en PCA 11):
//   HOME: adelante/atras en 90 y lateral en 100 (el cero del lateral es 100) -> los dos servos en 100
//   Adelante/atras (flex 6): los dos giran lo mismo en sentido INVERSO:  A = 90 + x,  B = 90 - x
//                            A va de 70 (atras) a 110 (adelante)
//   Lateral (flex 7 y 8):    los dos giran lo mismo en el MISMO sentido: A = B = 90 + y (y = lateral - 90)
//                            de 76 (lado del pulgar) a 120 (lado del menique)
//   Combinado:               A = 90 + x + y,  B = 90 - x + y
//   (misma convencion que "Arduino prueba colo/prueba_diferencial_muneca": modo I y modo M)
//   En la tabla son los "servos" 9 (adelante/atras) y 10 (lateral): s 9, m 10 80, etc.
//
// PROTECCION POR BAJA TENSION (igual que el receptor: modulo 0-25 V en GPIO34, mide V+ de los servos):
//   < 4.6 V  aviso por el monitor
//   < 4.3 V  CRITICA: corta la prueba y lleva todo a HOME despacio
//   < 3.9 V  CORTE: servos SUELTOS (con tan poca tension pueden saltar a cualquier lado)
//   (tiene que estar por debajo 150 ms seguidos: el pico de un instante al arrancar un servo no cuenta)
//   Despues de una falla no se mueve nada hasta mandar h, y h solo rearma con mas de 4.8 V.
//   Si al encender no hay tension (fuente apagada), la proteccion se activa sola al detectar > 4.8 V.
//   v        -> ver la tension;  v 5.12 = corregir la medicion con lo que marca el tester
//               (usa y guarda el mismo factor que el receptor)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define PCA_DIRECCION 0x40

// Pulso para 0 y 180 grados. 600-2400 us = la escala de las pruebas con el Arduino.
// (El receptor usa 400-2600 en los MG90S y 500-2500 en los MG92B: los mismos "grados" quedan en
// otro lugar. Si los rangos se midieron con el receptor, cambiar aca.)
#define US_MIN 600
#define US_MAX 2400
// DS3225 de la muneca: 500 us = 0, 1500 us = 90 (centro), 2500 us = 180 (igual que la prueba del Arduino)
#define DS3225_MIN_US 500
#define DS3225_MAX_US 2500

// ---------- Muneca ----------
#define CANAL_MUNECA_A 10   // S11 DS3225 (servo A de la prueba del Arduino)
#define CANAL_MUNECA_B 11   // S12 DS3225 (servo B)
#define CENTRO_MUNECA  90
// Si al probar un movimiento va para el lado contrario, cambiar a true:
#define INVERTIR_ADELANTE false   // adelante/atras al reves (intercambia que servo sube y cual baja)
#define INVERTIR_LATERAL  false   // lateral al reves (pulgar <-> menique)
// Limite de CADA servo de la muneca. Los movimientos solos: adelante/atras 70-110, lateral 76-120
// (el lateral hacia el menique lleva los dos servos a 120, por eso el maximo es 120).
// Combinados (w, o 9 y 10 a la vez) pueden pedir mas: 110 adelante + 120 menique -> A = 140.
// Si se pasa, se achican los dos movimientos en la misma proporcion (la direccion se mantiene).
#define LIMITE_MUNECA_MIN 70
#define LIMITE_MUNECA_MAX 120
#define SIN_CANAL 255       // "servo" virtual de la muneca: no tiene una salida propia

// ---------- Proteccion por baja tension (mismos valores que el receptor) ----------
#define PIN_TENSION          34
#define DIVISOR_TENSION      ((30000.0f + 7500.0f) / 7500.0f)   // modulo 0-25 V: divide por 5
#define TENSION_AVISO        4.6f
#define TENSION_CRITICA      4.3f   // corta la prueba y va a HOME
#define TENSION_CORTE        3.9f   // servos sueltos
#define TENSION_RECUPERADA   4.8f   // para rearmar con h (y para activar la proteccion si arranco sin fuente)
#define TENSION_SIN_DIVISOR  2.0f   // menos que esto = fuente apagada o sensor desconectado
#define TIEMPO_CRITICO_MS    150
#define MUESTRAS_TENSION     8
#define FILTRO_TENSION       0.1f   // ~50 ms

#define VELOCIDAD_INICIAL 40.0f   // grados por segundo
#define PAUSA_EXTREMO_MS  800     // espera en cada extremo del barrido
#define PERIODO_MS        10      // actualizacion de los servos (100 por segundo)

typedef struct {
  const char *dedo;
  const char *movimiento;
  const char *servo;
  uint8_t  canal;            // salida del PCA9685 (SIN_CANAL = muneca, ver escribirMuneca)
  uint16_t minUs, maxUs;     // pulso para 0 y 180 grados
  float    angMin, angMax;   // dedos: min = extendido, max = flexionado
  float    angHome;
  const char *haciaMin, *haciaMax;   // que hace yendo al minimo / al maximo (para los mensajes)
} ServoPrueba;

#define CANT_SERVOS 12
ServoPrueba SERVOS[CANT_SERVOS] = {
  //  dedo      movimiento   servo       PCA        pulso           min  max  HOME  hacia el min       hacia el max
  {"Pulgar", "Flex 1",     "S1 MG90S",   0,        US_MIN, US_MAX,   0, 180,   0, "extendiendo",      "flexionando"},       // 1
  {"Pulgar", "Nudillo",    "S6 MG90S",   5,        US_MIN, US_MAX,   0, 180,   0, "extendiendo",      "flexionando"},       // 2
  {"Pulgar", "Apertura",   "S13 MG92B", 12,        US_MIN, US_MAX,  11, 110,  11, "extendiendo",      "hacia la palma"},    // 3
  {"Indice", "Flex 2",     "S2 MG90S",   1,        US_MIN, US_MAX,   0, 180,   0, "extendiendo",      "flexionando"},       // 4
  {"Indice", "Nudillo",    "S7 MG90S",   6,        US_MIN, US_MAX,   0,  90,   0, "extendiendo",      "flexionando"},       // 5
  {"Indice", "Lateral",    "S14 MG92B", 13,        US_MIN, US_MAX,   0, 120,   0, "abriendo a la izq.", "cerrando a la der."}, // 6
  {"Medio",  "Flex 3",     "S3 MG90S",   2,        US_MIN, US_MAX,   0, 180,   0, "extendiendo",      "flexionando"},       // 7
  {"Medio",  "Nudillo",    "S8 MG90S",   7,        US_MIN, US_MAX,   0,  80,   0, "extendiendo",      "flexionando"},       // 8
  // Muneca: angulo del movimiento (no de un servo); los dos DS3225 salen de la combinacion de 9 y 10
  {"Muneca", "Adel/atras", "S11+S12",    SIN_CANAL, DS3225_MIN_US, DS3225_MAX_US, 70, 110, 90, "hacia atras",  "hacia adelante"},  // 9  (flex 6)
  {"Muneca", "Lateral",    "S11+S12",    SIN_CANAL, DS3225_MIN_US, DS3225_MAX_US, 76, 120, 100, "hacia el pulgar", "hacia el menique"}, // 10 (flex 7 y 8)
  // Anular: mismo rango que el medio (sin el lateral S15 / PCA 14). Va al final para no cambiar los numeros de la muneca
  {"Anular", "Flex 4",     "S4 MG90S",   3,        US_MIN, US_MAX,   0, 180,   0, "extendiendo",      "flexionando"},       // 11
  {"Anular", "Nudillo",    "S9 MG90S",   8,        US_MIN, US_MAX,   0,  80,   0, "extendiendo",      "flexionando"},       // 12
};
#define NUDILLO_PULGAR  1   // indices en SERVOS[]
#define APERTURA_PULGAR 2
#define LATERAL_INDICE  5
#define MUNECA_ADELANTE 8
#define MUNECA_LATERAL  9

// PULGAR: si se flexiona con la apertura cerrada (en su minimo, 11) choca con la palma. Por eso la apertura se
// mueve PRIMERO a este angulo y se queda ahi mientras se flexionan Flex 1 y el nudillo; al volver,
// la apertura vuelve ULTIMA. Vale para la prueba 1, s 1, s 2, d 1 y la vuelta a HOME.
// (m mueve solo lo que se le pide: ahi el orden lo elegis vos)
#define APERTURA_PULGAR_AL_FLEXIONAR 110

typedef struct { const char *nombre; uint8_t primero, cantidad; } Dedo;
Dedo DEDOS[] = { {"Pulgar", 0, 3}, {"Indice", 3, 3}, {"Medio", 6, 2}, {"Anular", 10, 2} };
#define CANT_DEDOS 4
Dedo MUNECA = {"Muneca", MUNECA_ADELANTE, 2};

// Servos que se mueven al cerrar/abrir el dedo entero (comando d). El lateral del indice
// es un movimiento hacia el costado, no de cierre: queda donde esta.
bool participaEnCierre(int i) { return i != LATERAL_INDICE; }

// ---------- Compensacion del pulgar ----------
// Fisicamente: el tendon (o la varilla) del nudillo pasa por la base del pulgar. Al girar la apertura,
// ese tendon se estira/afloja en proporcion al giro (como una polea: radio de la base x angulo), y la
// falange se cae esa misma proporcion. Por eso la correccion es LINEAL:
//     servo del nudillo = angulo pedido + K x (apertura - apertura minima - ZONA_SIN_COMPENSACION)
// y se aplica en la MISMA actualizacion que mueve la apertura (no tiene velocidad propia): el nudillo
// acompana a la apertura en todo momento, no llega tarde. Si al sumar se pasa de 180, queda en 180
// (con la apertura muy abierta no se llega a flexionar del todo: se avisa).
// ZONA SIN COMPENSACION: los primeros 30 gr de apertura (11 -> 41) no compensan. Si el nudillo se
// cerrara desde el principio, el pulgar choca con la palma antes de que la apertura lo saque afuera.
// K se calibra con g y se guarda en la memoria (NVS "mano", "kPulgar2"; la clave vieja "kPulgar"
// era sin zona muerta y ya no se usa).
#define ZONA_SIN_COMPENSACION  30.0f  // grados de apertura (desde su minimo) sin compensar
#define K_PULGAR_INICIAL       0.43f  // si nunca se calibro: con la apertura en 110, +30 gr de nudillo
#define K_PULGAR_MAXIMO        2.0f
#define COMPENSACION_AL_INICIO true
#define APERTURA_MINIMA_CAL    20.0f  // para calibrar: al menos 20 gr de apertura PASADA la zona sin compensar

float kPulgar = K_PULGAR_INICIAL;     // K elegido
float kActual = 0;                    // K aplicado: va hacia kPulgar (o 0) de a poco, sin saltos
bool compensacion = COMPENSACION_AL_INICIO;
bool avisoCompTope = false;
float nudilloConCompensacion(float a);

// ---------- Estado ----------
Adafruit_PWMServoDriver pca(PCA_DIRECCION);
float angulo[CANT_SERVOS];     // angulo actual de cada articulacion (el nudillo del pulgar SIN la
                               // compensacion: se suma al escribir el servo)
float objetivo[CANT_SERVOS];   // angulo pedido
float velocidad = VELOCIDAD_INICIAL;
float escalaVel[CANT_SERVOS];  // moviendo un dedo entero: cada servo va mas lento segun su recorrido
                               // para que todas las articulaciones lleguen a la vez (si no, 1)
bool activos = false;          // false = servos sueltos (sin pulso)

// Tension
Preferences memoria;
float tension = 0;              // V+ de los servos (filtrada)
float factorTension = 1.0f;     // correccion guardada por el receptor (comando v)
bool proteccionActiva = false;  // false = no se detecta tension (fuente apagada o sensor desconectado)
bool fallaTension = false;      // hubo tension critica o de corte: nada se mueve hasta rearmar con h
bool avisoTension = false;

// Secuencia de barrido: lista de pasos
// servo >= 0: ese servo a ang | servo -1: todos a HOME | dedo >= 0: ese dedo entero al ang %
// sinApertura (paso de dedo): el pulgar se mueve sin tocar la apertura (al abrir, la apertura vuelve despues)
typedef struct { int8_t servo; float ang; int8_t dedo; bool sinApertura; } Paso;
#define PASO_HOME              -1   // todos a HOME
#define PASO_HOME_SIN_APERTURA -2   // todos a HOME menos la apertura del pulgar (vuelve en el paso siguiente)
void agregarHome();
void iniciarSecuencia();
bool pasoMovio = false;        // el paso actual movio algo (si no, se saltea la pausa)
#define MAX_PASOS 32
Paso pasos[MAX_PASOS];
int cantPasos = 0, pasoActual = -1;
bool enPausa = false;
uint32_t tFinPaso = 0;

char linea[32];
int largoLinea = 0;

// =====================================================================
// PCA9685
// =====================================================================
void resetSoftwarePCA() {   // llamada general I2C 0x00 + 0x06: deja el PCA como recien encendido
  Wire.beginTransmission(0x00);
  Wire.write(0x06);
  Wire.endTransmission();
  delay(2);
}

int leerRegistroPCA(uint8_t reg) {
  Wire.beginTransmission(PCA_DIRECCION);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(PCA_DIRECCION, 1) != 1) return -1;
  return Wire.read();
}

int prescaleEsperado = -1;

void configurarPCA() {
  resetSoftwarePCA();
  pca.begin();
  pca.setPWMFreq(50);
  delay(10);
  for (int canal = 0; canal < 16; canal++) pca.setPWM(canal, 0, 4096);   // todo apagado (suelto)
  prescaleEsperado = leerRegistroPCA(0xFE);
}

void escribirPulso(uint8_t canal, uint16_t minUs, uint16_t maxUs, float a) {
  a = constrain(a, 0.0f, 180.0f);
  float us = minUs + a * (maxUs - minUs) / 180.0f;
  pca.setPWM(canal, 0, (uint16_t)lroundf(us * 4096.0f / 20000.0f));
}

// ---------- Muneca: de (adelante/atras, lateral) a los dos DS3225 ----------
float anguloA = CENTRO_MUNECA, anguloB = CENTRO_MUNECA;   // ultimo enviado a cada servo (para e)
bool munecaLimitada = false;

// Calcula A y B con los angulos de los "servos" 9 y 10 (sin escribir)
void calcularMuneca(float adelante, float lateral, float &a, float &b, bool &limitada) {
  float x = adelante - CENTRO_MUNECA;   // + adelante
  float y = lateral - CENTRO_MUNECA;    // + hacia el menique
  if (INVERTIR_ADELANTE) x = -x;
  if (INVERTIR_LATERAL) y = -y;
  // Si algun servo se pasa del limite, se achican x e y juntos
  float k = 1.0f;
  float candidatos[2] = {x + y, -x + y};   // desplazamiento de A y de B respecto del centro
  for (int j = 0; j < 2; j++) {
    float d = candidatos[j];
    if (d > LIMITE_MUNECA_MAX - CENTRO_MUNECA) k = min(k, (LIMITE_MUNECA_MAX - CENTRO_MUNECA) / d);
    if (d < LIMITE_MUNECA_MIN - CENTRO_MUNECA) k = min(k, (LIMITE_MUNECA_MIN - CENTRO_MUNECA) / d);
  }
  limitada = k < 0.999f;
  a = CENTRO_MUNECA + k * (x + y);
  b = CENTRO_MUNECA + k * (-x + y);
}

void escribirMuneca() {
  bool limitada;
  calcularMuneca(angulo[MUNECA_ADELANTE], angulo[MUNECA_LATERAL], anguloA, anguloB, limitada);
  if (limitada && !munecaLimitada) {
    Serial.printf("  (muneca: el combinado pasa el limite %d-%d de los servos: se achica)\n",
                  LIMITE_MUNECA_MIN, LIMITE_MUNECA_MAX);
  }
  munecaLimitada = limitada;
  escribirPulso(CANAL_MUNECA_A, DS3225_MIN_US, DS3225_MAX_US, anguloA);
  escribirPulso(CANAL_MUNECA_B, DS3225_MIN_US, DS3225_MAX_US, anguloB);
}

void escribirServo(int i, float a) {
  if (!activos) return;
  const ServoPrueba &s = SERVOS[i];
  if (s.canal == SIN_CANAL) { escribirMuneca(); return; }   // usa angulo[] de 9 y 10
  if (i == NUDILLO_PULGAR) a = nudilloConCompensacion(a);
  escribirPulso(s.canal, s.minUs, s.maxUs, a);
}

// Saca el pulso de todos los servos (quedan sueltos, sin fuerza)
void apagarSalidas() {
  for (int i = 0; i < CANT_SERVOS; i++) {
    if (SERVOS[i].canal != SIN_CANAL) pca.setPWM(SERVOS[i].canal, 0, 4096);
  }
  pca.setPWM(CANAL_MUNECA_A, 0, 4096);
  pca.setPWM(CANAL_MUNECA_B, 0, 4096);
}

// Si el PCA quedo dormido o con el reloj externo (paso el 2026-11-09), se reconfigura
void revisarPCA() {
  static uint32_t t = 0, tAviso = 0;
  if (millis() - t < 1000) return;
  t = millis();
  int mode1 = leerRegistroPCA(0x00), prescale = leerRegistroPCA(0xFE);
  if (mode1 < 0 || prescale < 0) {
    if (millis() - tAviso > 5000) {
      tAviso = millis();
      Serial.println(">>> PCA9685 NO RESPONDE: revisar SDA (GPIO21), SCL (GPIO22), VCC y GND");
    }
    return;
  }
  if (!(mode1 & 0x50) && prescale == prescaleEsperado) return;
  Serial.printf(">>> PCA9685 se reinicio (MODE1 0x%02X): se vuelve a configurar\n", mode1);
  configurarPCA();
  if (activos) for (int i = 0; i < CANT_SERVOS; i++) escribirServo(i, angulo[i]);
}

// =====================================================================
// COMPENSACION DEL PULGAR
// =====================================================================
// Grados de apertura que cuentan para la compensacion: 0 hasta pasar la zona sin compensar
float aperturaRelativa() {
  return max(angulo[APERTURA_PULGAR] - SERVOS[APERTURA_PULGAR].angMin - ZONA_SIN_COMPENSACION, 0.0f);
}

// Grados que se suman al servo del nudillo con la apertura actual
float compensacionNudillo() { return kActual * aperturaRelativa(); }

// Angulo del servo del nudillo: lo pedido + la compensacion, dentro del rango del servo
float nudilloConCompensacion(float a) {
  const ServoPrueba &s = SERVOS[NUDILLO_PULGAR];
  float servo = a + compensacionNudillo();
  bool tope = servo > s.angMax + 0.05f;
  if (tope && !avisoCompTope) {
    Serial.printf("  (pulgar: nudillo %.0f + compensacion %.0f pasa de %.0f: queda en %.0f. Con la apertura tan"
                  " abierta no flexiona mas)\n", a, compensacionNudillo(), s.angMax, s.angMax);
  }
  avisoCompTope = tope;
  return constrain(servo, s.angMin, s.angMax);
}

// Lleva kActual hacia kPulgar (o a 0 si esta desactivada) sin que el nudillo vaya mas rapido que
// la velocidad elegida. Devuelve true si cambio.
bool actualizarKActual(float dt) {
  float destino = compensacion ? kPulgar : 0;
  float dif = destino - kActual;
  if (dif == 0) return false;
  if (fabsf(dif) < 0.0005f) { kActual = destino; return true; }
  float maxPaso = velocidad * dt / max(aperturaRelativa(), 1.0f);   // grados de nudillo por segundo
  kActual += constrain(dif, -maxPaso, maxPaso);
  return true;
}

void imprimirCompensacion() {
  Serial.printf("Compensacion del pulgar %s | K = %.3f (aplicado %.3f) | apertura %.0f -> nudillo +%.0f\n",
                compensacion ? "ACTIVADA" : "desactivada", kPulgar, kActual, angulo[APERTURA_PULGAR],
                compensacionNudillo());
  const ServoPrueba &ap = SERVOS[APERTURA_PULGAR];
  Serial.printf("  (empieza a compensar con la apertura en %.0f; en %.0f el nudillo se corre +%.0f)\n",
                ap.angMin + ZONA_SIN_COMPENSACION, ap.angMax,
                kPulgar * max(ap.angMax - ap.angMin - ZONA_SIN_COMPENSACION, 0.0f));
}

void guardarK() { memoria.putFloat("kPulgar2", kPulgar); }

// Mensaje al mover la apertura o el nudillo del pulgar: como va a quedar el servo del nudillo
void avisarCompensacion(int servo, float destino) {
  if (servo != APERTURA_PULGAR && servo != NUDILLO_PULGAR) return;
  if (!compensacion) { Serial.println("     (compensacion del pulgar desactivada: c para activarla)"); return; }
  const ServoPrueba &ap = SERVOS[APERTURA_PULGAR];
  float apFinal = servo == APERTURA_PULGAR ? destino : angulo[APERTURA_PULGAR];
  float nudFinal = servo == NUDILLO_PULGAR ? destino : angulo[NUDILLO_PULGAR];
  float comp = kPulgar * max(apFinal - ap.angMin - ZONA_SIN_COMPENSACION, 0.0f);
  float servoNud = constrain(nudFinal + comp, SERVOS[NUDILLO_PULGAR].angMin, SERVOS[NUDILLO_PULGAR].angMax);
  if (comp < 0.5f) {
    Serial.printf("     (compensacion: +0, la apertura en %.0f no pasa de %.0f)\n", apFinal,
                  ap.angMin + ZONA_SIN_COMPENSACION);
  } else {
    Serial.printf("     (compensacion: nudillo %.0f + %.0f -> servo del nudillo en %.0f%s)\n", nudFinal, comp,
                  servoNud, nudFinal + comp > SERVOS[NUDILLO_PULGAR].angMax ? ", en el tope" : "");
  }
}

// Comando g: con la apertura afuera y el nudillo ubicado a mano (m 2 A) donde la falange queda como
// un pulgar real extendido, calcula K = angulo del servo del nudillo / apertura relativa.
// Ese servo queda donde esta; desde ahora esa postura cuenta como "nudillo en 0".
void calibrarCompensacion() {
  float rel = aperturaRelativa();
  if (rel < APERTURA_MINIMA_CAL) {
    Serial.printf("Para calibrar, la apertura tiene que estar en %.0f o mas (por ejemplo m 3 110)."
                  " Despues ubicar el nudillo con m 2 A y mandar g.\n",
                  SERVOS[APERTURA_PULGAR].angMin + ZONA_SIN_COMPENSACION + APERTURA_MINIMA_CAL);
    return;
  }
  if (enMovimiento()) { Serial.println("Esperar a que termine el movimiento."); return; }
  float servoNudillo = angulo[NUDILLO_PULGAR] + compensacionNudillo();
  float k = servoNudillo / rel;
  if (k > K_PULGAR_MAXIMO) { Serial.printf("K = %.2f es demasiado (maximo %.1f): revisar.\n", k, K_PULGAR_MAXIMO); return; }
  kPulgar = kActual = k;
  compensacion = true;
  angulo[NUDILLO_PULGAR] = objetivo[NUDILLO_PULGAR] = SERVOS[NUDILLO_PULGAR].angMin;   // esta postura = nudillo en 0
  escribirServo(NUDILLO_PULGAR, angulo[NUDILLO_PULGAR]);   // el servo no se mueve (misma cuenta)
  guardarK();
  Serial.printf(">>> Compensacion calibrada y guardada: K = %.3f (servo del nudillo en %.0f con la apertura en %.0f)\n",
                k, servoNudillo, angulo[APERTURA_PULGAR]);
  imprimirCompensacion();
}

// Angulo al que tiene que ir el servo i (limitado a su rango)
float objetivoEfectivo(int i) { return constrain(objetivo[i], SERVOS[i].angMin, SERVOS[i].angMax); }

// =====================================================================
// MOVIMIENTO
// =====================================================================
bool enMovimiento() {
  if (!activos) return false;
  if (pasoActual >= 0) return true;
  if ((compensacion ? kPulgar : 0) != kActual) return true;
  for (int i = 0; i < CANT_SERVOS; i++) {
    if (fabsf(objetivoEfectivo(i) - angulo[i]) > 0.05f) return true;
  }
  return false;
}

void actualizarServos() {
  static uint32_t tAnterior = 0;
  uint32_t ahora = millis();
  if (ahora - tAnterior < PERIODO_MS) return;
  float dt = (ahora - tAnterior) / 1000.0f;
  tAnterior = ahora;
  if (!activos) return;
  if (dt > 0.1f) dt = 0.1f;
  bool reescribirNudillo = actualizarKActual(dt);
  for (int i = 0; i < CANT_SERVOS; i++) {
    float dif = objetivoEfectivo(i) - angulo[i];
    if (fabsf(dif) < 0.05f) continue;
    float maxPaso = velocidad * escalaVel[i] * dt;
    angulo[i] += constrain(dif, -maxPaso, maxPaso);
    escribirServo(i, angulo[i]);
    if (i == APERTURA_PULGAR) reescribirNudillo = true;
  }
  // El nudillo acompana a la apertura en la misma actualizacion (acople: no llega tarde)
  if (reescribirNudillo) escribirServo(NUDILLO_PULGAR, angulo[NUDILLO_PULGAR]);
}

void parada(const char *motivo) {
  cantPasos = 0;
  pasoActual = -1;
  enPausa = false;
  apagarSalidas();   // sin pulso: servos sueltos
  for (int i = 0; i < CANT_SERVOS; i++) objetivo[i] = angulo[i];
  activos = false;
  Serial.printf("\n>>> PARADA DE EMERGENCIA (%s): servos SUELTOS. Para seguir: h\n", motivo);
}

void velocidadNormal() {
  for (int i = 0; i < CANT_SERVOS; i++) escalaVel[i] = 1.0f;
}

// Los servos desde 'primero' (cantidad) que tengan objetivo nuevo arrancan y llegan juntos:
// el de mas recorrido va a la velocidad elegida, los otros mas lento.
void sincronizar(int primero, int cantidad) {
  float recorridoMax = 0;
  for (int j = primero; j < primero + cantidad; j++) recorridoMax = max(recorridoMax, fabsf(objetivo[j] - angulo[j]));
  velocidadNormal();
  if (recorridoMax < 0.05f) return;
  for (int j = primero; j < primero + cantidad; j++) {
    escalaVel[j] = max(fabsf(objetivo[j] - angulo[j]) / recorridoMax, 0.05f);
  }
}

// Lleva el dedo d entero al pct % (0 = extendido, 100 = flexionado), todas las articulaciones juntas
// sinApertura: la apertura del pulgar queda donde esta
void moverDedo(int d, float pct, bool sinApertura = false) {
  const Dedo &dd = DEDOS[d];
  for (int j = dd.primero; j < dd.primero + dd.cantidad; j++) {
    if (!participaEnCierre(j) || (sinApertura && j == APERTURA_PULGAR)) continue;
    objetivo[j] = SERVOS[j].angMin + pct / 100.0f * (SERVOS[j].angMax - SERVOS[j].angMin);
  }
  sincronizar(dd.primero, dd.cantidad);
  Serial.printf("  %s al %.0f %%:", dd.nombre, pct);
  for (int j = dd.primero; j < dd.primero + dd.cantidad; j++) {
    if (participaEnCierre(j) && !(sinApertura && j == APERTURA_PULGAR))
      Serial.printf("  %s %.0f gr", SERVOS[j].movimiento, objetivo[j]);
  }
  Serial.println();
}

void irAHome() {
  if (fallaTension) {
    if (tension < TENSION_RECUPERADA) {
      Serial.printf(">>> TENSION: %.2f V, todavia baja (hace falta mas de %.1f V). No se rearma.\n",
                    tension, TENSION_RECUPERADA);
      return;
    }
    fallaTension = false;
    Serial.printf(">>> TENSION: rearmado con %.2f V\n", tension);
  }
  if (!proteccionActiva) {
    Serial.println("ATENCION: proteccion de tension INACTIVA (no se mide tension en los servos: fuente apagada o sensor desconectado)");
  }
  cantPasos = 0;
  pasoActual = -1;
  velocidadNormal();
  if (!activos) {
    // Vuelven a tener pulso donde quedaron (si se movieron a mano estando sueltos, dan un salto)
    activos = true;
    for (int i = 0; i < CANT_SERVOS; i++) escribirServo(i, angulo[i]);
    Serial.println("Servos ACTIVADOS.");
  }
  Serial.printf("Yendo a HOME (dedos extendidos, muneca al centro; la apertura del pulgar vuelve ultima) a %.0f gr/s.\n",
                velocidad);
  agregarHome();
  iniciarSecuencia();
}

// =====================================================================
// PROTECCION POR BAJA TENSION
// =====================================================================
// Promedia varias muestras del ADC: una sola tiene unos ±0.5 V de ruido
float leerTension() {
  uint32_t suma = 0;
  for (int k = 0; k < MUESTRAS_TENSION; k++) suma += analogReadMilliVolts(PIN_TENSION);
  return suma / (float)MUESTRAS_TENSION / 1000.0f * DIVISOR_TENSION * factorTension;
}

// Se llama seguido desde loop(): mide V+ (filtrada) y actua por niveles
void revisarTension() {
  static uint32_t tMedicion = 0, tBajoCritica = 0, tBajoCorte = 0;
  uint32_t ahora = millis();
  if (ahora - tMedicion < 5) return;
  tMedicion = ahora;

  float v = leerTension();
  tension = tension == 0 ? v : tension + FILTRO_TENSION * (v - tension);

  if (!proteccionActiva) {
    // Arranco sin tension: se activa cuando aparece la fuente (ya estable, > 4.8 V)
    if (tension >= TENSION_RECUPERADA) {
      proteccionActiva = true;
      tBajoCritica = tBajoCorte = 0;
      Serial.printf(">>> TENSION: fuente detectada (%.2f V): PROTECCION ACTIVA\n", tension);
    }
    return;
  }

  // Corte: servos sueltos
  if (tension < TENSION_CORTE) {
    if (!tBajoCorte) tBajoCorte = ahora;
    if (activos && ahora - tBajoCorte >= TIEMPO_CRITICO_MS) {
      cantPasos = 0;
      pasoActual = -1;
      enPausa = false;
      apagarSalidas();
      for (int i = 0; i < CANT_SERVOS; i++) objetivo[i] = angulo[i];
      activos = false;
      fallaTension = true;
      Serial.printf("\n>>> TENSION DE CORTE: %.2f V (< %.1f V). SERVOS SUELTOS (sin fuerza).\n", tension, TENSION_CORTE);
      Serial.println(">>> Revisar la fuente. Con la tension recuperada (> 4.8 V), mandar h para rearmar.");
    }
  } else {
    tBajoCorte = 0;
  }

  // Critica: corta la prueba y va a HOME
  if (tension < TENSION_CRITICA) {
    if (!tBajoCritica) tBajoCritica = ahora;
    if (!fallaTension && ahora - tBajoCritica >= TIEMPO_CRITICO_MS) {
      fallaTension = true;
      cantPasos = 0;
      pasoActual = -1;
      enPausa = false;
      if (activos) agregarHome();   // la apertura del pulgar vuelve ultima (si no, choca con la palma)
      Serial.printf("\n>>> TENSION CRITICA: %.2f V (< %.1f V). Prueba cortada: todo a HOME.\n", tension, TENSION_CRITICA);
      Serial.println(">>> Revisar la fuente. Con la tension recuperada (> 4.8 V), mandar h para rearmar.");
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
}

// Comando v: muestra la tension; "v 5.12" corrige la medicion con lo que marca el tester
void comandoTension(const char *args) {
  float real;
  if (sscanf(args, "%f", &real) == 1) {
    float medida = 0;
    for (int k = 0; k < 20; k++) { medida += leerTension(); delay(2); }
    medida /= 20;
    if (medida < TENSION_SIN_DIVISOR) { Serial.println(">>> TENSION: no se mide tension (fuente apagada o sensor desconectado)"); return; }
    factorTension *= real / medida;
    memoria.putFloat("factorV", factorTension);
    tension = real;
    Serial.printf(">>> TENSION: corregida (factor %.3f) y guardada\n", factorTension);
  }
  Serial.printf(">>> TENSION servos: %.2f V | aviso < %.1f | critica < %.1f (HOME) | corte < %.1f (sueltos) | factor %.3f\n",
                tension, TENSION_AVISO, TENSION_CRITICA, TENSION_CORTE, factorTension);
  Serial.printf("    Proteccion %s%s\n", proteccionActiva ? "ACTIVA" : "INACTIVA (se activa sola con > 4.8 V)",
                fallaTension ? " | EN FALLA (h para rearmar)" : "");
}

// ---------- Secuencia ----------
void agregarPaso(int8_t servo, float ang) {
  if (cantPasos < MAX_PASOS) pasos[cantPasos++] = {servo, ang, -1, false};
}

void agregarPasoDedo(int8_t dedo, float pct, bool sinApertura = false) {
  if (cantPasos < MAX_PASOS) pasos[cantPasos++] = {PASO_HOME, pct, dedo, sinApertura};
}

// Vuelta a HOME en dos pasos: primero todo menos la apertura del pulgar, despues la apertura
void agregarHome() {
  agregarPaso(PASO_HOME_SIN_APERTURA, 0);
  agregarPaso(PASO_HOME, 0);
}

// Barrido de Flex 1 (servo 0) o del nudillo del pulgar (servo 1) con la apertura afuera
void agregarBarridoPulgarConApertura(int i) {
  agregarPaso(APERTURA_PULGAR, APERTURA_PULGAR_AL_FLEXIONAR);
  agregarBarrido(i);
  agregarPaso(APERTURA_PULGAR, SERVOS[APERTURA_PULGAR].angMin);
}

// Dedos: min -> max -> min.  Muneca (HOME en el centro): centro -> max -> min -> centro
void agregarBarrido(int i) {
  agregarPaso(i, SERVOS[i].angMax);
  agregarPaso(i, SERVOS[i].angMin);
  if (SERVOS[i].angHome != SERVOS[i].angMin) agregarPaso(i, SERVOS[i].angHome);
}

void iniciarSecuencia() {
  pasoActual = -1;
  enPausa = false;
  tFinPaso = 0;
  Serial.println("(x = PARADA DE EMERGENCIA)");
}

void empezarPaso(int k) {
  pasoActual = k;
  enPausa = false;
  const Paso &p = pasos[k];
  if (p.dedo >= 0) {
    moverDedo(p.dedo, p.ang, p.sinApertura);
  } else if (p.servo == PASO_HOME || p.servo == PASO_HOME_SIN_APERTURA) {
    for (int i = 0; i < CANT_SERVOS; i++) {
      if (p.servo == PASO_HOME_SIN_APERTURA && i == APERTURA_PULGAR) continue;
      objetivo[i] = SERVOS[i].angHome;
    }
    sincronizar(MUNECA_ADELANTE, 2);   // la muneca vuelve al centro en linea recta
    if (!servosLlegaron()) {
      Serial.println(p.servo == PASO_HOME ? "  Todos a HOME" : "  A HOME (la apertura del pulgar vuelve despues)");
    }
  } else {
    moverServoPaso(k);
  }
  pasoMovio = !servosLlegaron();
}

void moverServoPaso(int k) {   // paso k de la secuencia: mueve un servo
  const Paso &p = pasos[k];
  velocidadNormal();
  const ServoPrueba &s = SERVOS[p.servo];
  const char *sentido = p.ang > angulo[p.servo] ? s.haciaMax : s.haciaMin;
  if (fabsf(p.ang - s.angHome) < 0.05f && s.angHome != s.angMin) sentido = "al centro";
  if (s.canal == SIN_CANAL) {
    float a, b;
    bool lim;
    float adel = p.servo == MUNECA_ADELANTE ? p.ang : angulo[MUNECA_ADELANTE];
    float lat = p.servo == MUNECA_LATERAL ? p.ang : angulo[MUNECA_LATERAL];
    calcularMuneca(adel, lat, a, b, lim);
    Serial.printf("  %d. Muneca %s: %.0f -> %.0f gr (%s)  ->  A (PCA %d) %.0f, B (PCA %d) %.0f\n", p.servo + 1,
                  s.movimiento, angulo[p.servo], p.ang, sentido, CANAL_MUNECA_A, a, CANAL_MUNECA_B, b);
  } else {
    Serial.printf("  %d. %s %s (%s, PCA %d): %.0f -> %.0f gr (%s)\n", p.servo + 1, s.dedo, s.movimiento, s.servo,
                  s.canal, angulo[p.servo], p.ang, sentido);
    avisarCompensacion(p.servo, p.ang);
  }
  objetivo[p.servo] = p.ang;
}

bool servosLlegaron() {
  for (int i = 0; i < CANT_SERVOS; i++) {
    if (fabsf(objetivoEfectivo(i) - angulo[i]) > 0.05f) return false;
  }
  return true;
}

void actualizarSecuencia() {
  if (!activos || cantPasos == 0) return;
  if (pasoActual < 0) { empezarPaso(0); return; }
  if (!servosLlegaron()) return;
  if (pasoMovio) {   // pausa en el extremo (si el paso no movio nada, se saltea)
    if (!enPausa) { enPausa = true; tFinPaso = millis(); return; }
    if (millis() - tFinPaso < PAUSA_EXTREMO_MS) return;
  }
  if (pasoActual + 1 < cantPasos) {
    empezarPaso(pasoActual + 1);
  } else {
    cantPasos = 0;
    pasoActual = -1;
    Serial.println(">>> Listo.");
  }
}

void probarDedo(const Dedo &dd) {
  Serial.printf("=== %s ===\n", dd.nombre);
  if (dd.primero == 0) {
    // Pulgar: apertura primero (y se queda afuera), Flex 1, nudillo, y la apertura vuelve al final
    agregarPaso(APERTURA_PULGAR, APERTURA_PULGAR_AL_FLEXIONAR);
    agregarBarrido(0);
    agregarBarrido(1);
    agregarPaso(APERTURA_PULGAR, SERVOS[APERTURA_PULGAR].angMin);
    return;
  }
  for (int j = 0; j < dd.cantidad; j++) agregarBarrido(dd.primero + j);
}

// =====================================================================
// COMANDOS
// =====================================================================
void imprimirEstado() {
  Serial.printf("Servos %s | velocidad %.0f gr/s\n", activos ? "ACTIVOS" : "SUELTOS", velocidad);
  imprimirCompensacion();
  Serial.printf("  Servo del nudillo del pulgar: %.1f (pedido %.1f + compensacion %.1f)\n",
                nudilloConCompensacion(angulo[NUDILLO_PULGAR]), angulo[NUDILLO_PULGAR], compensacionNudillo());
  Serial.printf("Tension servos %.2f V | proteccion %s%s\n", tension, proteccionActiva ? "ACTIVA" : "INACTIVA",
                fallaTension ? " | EN FALLA (h para rearmar)" : "");
  Serial.println("   N  dedo    movimiento  servo      PCA  rango      HOME  angulo");
  for (int i = 0; i < CANT_SERVOS; i++) {
    const ServoPrueba &s = SERVOS[i];
    char pcaTxt[8];
    if (s.canal == SIN_CANAL) snprintf(pcaTxt, sizeof(pcaTxt), "%d+%d", CANAL_MUNECA_A, CANAL_MUNECA_B);
    else snprintf(pcaTxt, sizeof(pcaTxt), "%d", s.canal);
    Serial.printf("  %2d  %-7s %-11s %-10s %5s  %3.0f - %3.0f  %4.0f  %5.1f\n", i + 1, s.dedo, s.movimiento,
                  s.servo, pcaTxt, s.angMin, s.angMax, s.angHome, angulo[i]);
  }
  Serial.printf("  Servos de la muneca: A (S11, PCA %d) = %.1f   B (S12, PCA %d) = %.1f   (limite %d-%d%s)\n",
                CANAL_MUNECA_A, anguloA, CANAL_MUNECA_B, anguloB, LIMITE_MUNECA_MIN, LIMITE_MUNECA_MAX,
                munecaLimitada ? ", LIMITADO" : "");
}

void imprimirAyuda() {
  Serial.println();
  Serial.println("===== PRUEBA DE DEDOS Y MUNECA (sin guante) =====");
  Serial.println("  h       activar servos e ir a HOME (mano extendida, muneca al centro)");
  Serial.println("  1 2 3 4 probar pulgar / indice / medio / anular (de a un servo, min -> max -> min)");
  Serial.println("  0       probar los 4 dedos seguidos");
  Serial.println("  5       probar la muneca (adelante/atras y despues lateral)");
  Serial.println("  w A L   muneca: adelante/atras A (70 atras..110 adelante), lateral L (76 pulgar..120 menique)");
  Serial.println("  d N     mover el dedo N ENTERO (1 pulgar, 2 indice, 3 medio, 4 anular): cerrar y abrir");
  Serial.println("  d N P   llevar el dedo N al P % (0 extendido, 100 flexionado)");
  Serial.println("  s N     barrer solo el servo N (1-12; 9 muneca adelante/atras, 10 muneca lateral, 11-12 anular)");
  Serial.println("  m N A   mover el servo N al angulo A");
  Serial.println("  r N     rapidez (velocidad) en gr/s");
  Serial.println("  e       estado / tabla de servos");
  Serial.println("  v       tension de los servos (v 5.12 = corregir con el tester)");
  Serial.println("  x       PARADA DE EMERGENCIA (servos sueltos), en cualquier momento");
  Serial.println("  Compensacion del pulgar: c on/off | k ver K | k 0.4 cambiar K | g calibrar (apertura afuera + nudillo a mano)");
  Serial.println();
}

bool verificarActivos() {
  if (fallaTension) {
    Serial.printf("Falla de tension (%.2f V): no se mueve nada. Con mas de %.1f V, mandar h para rearmar.\n",
                  tension, TENSION_RECUPERADA);
    return false;
  }
  if (activos) return true;
  Serial.println("Servos sueltos: mandar h primero.");
  return false;
}

bool servoValido(int n) {
  if (n >= 1 && n <= CANT_SERVOS) return true;
  Serial.printf("Servo invalido (1 a %d, ver e).\n", CANT_SERVOS);
  return false;
}

void ejecutarComando(char *cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == 0) return;
  char c = tolower(cmd[0]);
  int n = 0;
  float a = 0;

  switch (c) {
    case 'h': irAHome(); break;
    case 'x': parada("x"); break;
    case '?': imprimirAyuda(); break;
    case 'e': imprimirEstado(); break;

    case '0': case '1': case '2': case '3': case '4': case '5':
      if (!verificarActivos()) break;
      cantPasos = 0;
      agregarHome();   // primero todos a HOME
      if (c == '0') for (int d = 0; d < CANT_DEDOS; d++) probarDedo(DEDOS[d]);
      else if (c == '5') probarDedo(MUNECA);
      else probarDedo(DEDOS[c - '1']);
      iniciarSecuencia();
      break;

    case 'w': {
      float lat = 0;
      if (sscanf(cmd + 1, "%f %f", &a, &lat) != 2) { Serial.println("Uso: w A L   (ej: w 100 100)"); break; }
      if (!verificarActivos()) break;
      const ServoPrueba &sa = SERVOS[MUNECA_ADELANTE], &sl = SERVOS[MUNECA_LATERAL];
      if (a < sa.angMin || a > sa.angMax || lat < sl.angMin || lat > sl.angMax) {
        a = constrain(a, sa.angMin, sa.angMax);
        lat = constrain(lat, sl.angMin, sl.angMax);
        Serial.printf("Fuera de rango: se limita a %.0f %.0f\n", a, lat);
      }
      cantPasos = 0;
      pasoActual = -1;
      objetivo[MUNECA_ADELANTE] = a;
      objetivo[MUNECA_LATERAL] = lat;
      sincronizar(MUNECA_ADELANTE, 2);   // los dos movimientos llegan juntos
      float pa, pb;
      bool lim;
      calcularMuneca(a, lat, pa, pb, lim);
      Serial.printf("Muneca -> adelante/atras %.0f, lateral %.0f  =>  A %.0f, B %.0f%s (x = PARADA)\n",
                    a, lat, pa, pb, lim ? " (limitado)" : "");
      break;
    }

    case 'd': {
      int leidos = sscanf(cmd + 1, "%d %f", &n, &a);
      if (leidos < 1 || n < 1 || n > CANT_DEDOS) { Serial.println("Uso: d N  o  d N P   (N: 1 pulgar, 2 indice, 3 medio, 4 anular)"); break; }
      if (!verificarActivos()) break;
      cantPasos = 0;
      pasoActual = -1;
      if (leidos == 1) {
        Serial.printf("=== %s entero: cerrar y abrir ===\n", DEDOS[n - 1].nombre);
        agregarHome();               // primero todos a HOME
        if (n == 1) {
          // Pulgar: la apertura sale primero y vuelve ultima (si no, choca con la palma)
          agregarPaso(APERTURA_PULGAR, SERVOS[APERTURA_PULGAR].angMax);
          agregarPasoDedo(0, 100);
          agregarPasoDedo(0, 0, true);
          agregarPaso(APERTURA_PULGAR, SERVOS[APERTURA_PULGAR].angMin);
        } else {
          agregarPasoDedo(n - 1, 100);
          agregarPasoDedo(n - 1, 0);
        }
        iniciarSecuencia();
      } else {
        if (a < 0 || a > 100) {
          a = constrain(a, 0.0f, 100.0f);
          Serial.printf("Porcentaje fuera de rango: se limita a %.0f\n", a);
        }
        if (n == 1) {
          // Pulgar: cerrando, primero la apertura; abriendo, la apertura al final
          const ServoPrueba &ap = SERVOS[APERTURA_PULGAR];
          float destinoAp = ap.angMin + a / 100.0f * (ap.angMax - ap.angMin);
          if (destinoAp >= angulo[APERTURA_PULGAR]) {
            agregarPaso(APERTURA_PULGAR, destinoAp);
            agregarPasoDedo(0, a);
          } else {
            agregarPasoDedo(0, a, true);
            agregarPaso(APERTURA_PULGAR, destinoAp);
          }
          iniciarSecuencia();
        } else {
          moverDedo(n - 1, a);
          Serial.println("  (x = PARADA)");
        }
      }
      break;
    }

    case 's':
      if (sscanf(cmd + 1, "%d", &n) != 1) { Serial.println("Uso: s N"); break; }
      if (!servoValido(n) || !verificarActivos()) break;
      cantPasos = 0;
      agregarHome();
      if (n - 1 == 0 || n - 1 == NUDILLO_PULGAR) agregarBarridoPulgarConApertura(n - 1);   // Flex 1 o nudillo del pulgar
      else agregarBarrido(n - 1);
      iniciarSecuencia();
      break;

    case 'm':
      if (sscanf(cmd + 1, "%d %f", &n, &a) != 2) { Serial.println("Uso: m N A   (ej: m 3 60)"); break; }
      if (!servoValido(n) || !verificarActivos()) break;
      if (a < SERVOS[n - 1].angMin || a > SERVOS[n - 1].angMax) {
        a = constrain(a, SERVOS[n - 1].angMin, SERVOS[n - 1].angMax);
        Serial.printf("Fuera de rango: se limita a %.0f\n", a);
      }
      cantPasos = 0;
      pasoActual = -1;
      velocidadNormal();
      objetivo[n - 1] = a;
      Serial.printf("%d. %s %s -> %.0f gr (x = PARADA)\n", n, SERVOS[n - 1].dedo,
                    SERVOS[n - 1].movimiento, a);
      avisarCompensacion(n - 1, a);
      break;

    case 'v': comandoTension(cmd + 1); break;

    case 'r':
      if (sscanf(cmd + 1, "%f", &a) != 1 || a < 5 || a > 300) { Serial.println("Uso: r N  (5 a 300 gr/s)"); break; }
      velocidad = a;
      Serial.printf("Velocidad: %.0f gr/s\n", velocidad);
      break;

    case 'c':
      compensacion = !compensacion;
      Serial.printf("Compensacion del pulgar %s (cambia de a poco)\n", compensacion ? "ACTIVADA" : "desactivada");
      imprimirCompensacion();
      break;
    case 'g': calibrarCompensacion(); break;
    case 'k':
      if (sscanf(cmd + 1, "%f", &a) == 1) {
        if (a < 0 || a > K_PULGAR_MAXIMO) { Serial.printf("K entre 0 y %.1f\n", K_PULGAR_MAXIMO); break; }
        kPulgar = a;
        guardarK();
        Serial.println("K cambiado y guardado (el nudillo se acomoda de a poco).");
      }
      imprimirCompensacion();
      break;

    default:
      Serial.printf("Comando desconocido: %s  (? = ayuda)\n", cmd);
  }
}

void leerSerie() {
  while (Serial.available()) {
    char ch = Serial.read();
    // PARADA: solo la x, en cualquier momento y sin esperar Enter (ningun comando lleva x)
    if (ch == 'x' || ch == 'X') {
      parada("x");
      largoLinea = 0;
      continue;
    }
    if (ch == '\r' || ch == '\n') {
      linea[largoLinea] = 0;
      ejecutarComando(linea);
      largoLinea = 0;
    } else if (largoLinea < (int)sizeof(linea) - 1) {
      linea[largoLinea++] = ch;
    }
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  configurarPCA();
  if (prescaleEsperado < 0) Serial.println(">>> PCA9685 NO RESPONDE: revisar SDA (GPIO21), SCL (GPIO22), VCC y GND");

  for (int i = 0; i < CANT_SERVOS; i++) angulo[i] = objetivo[i] = SERVOS[i].angHome;
  velocidadNormal();

  // Proteccion por baja tension: mismo factor de calibracion que guardo el receptor
  memoria.begin("mano", false);
  factorTension = memoria.getFloat("factorV", 1.0f);
  kPulgar = memoria.getFloat("kPulgar2", K_PULGAR_INICIAL);
  kActual = compensacion ? kPulgar : 0;   // al arrancar la apertura esta en su minimo: no mueve nada
  analogSetPinAttenuation(PIN_TENSION, ADC_11db);   // rango hasta ~3.1 V en el pin
  for (int k = 0; k < 20; k++) { tension += leerTension(); delay(2); }
  tension /= 20;
  proteccionActiva = tension >= TENSION_SIN_DIVISOR;

  imprimirAyuda();
  imprimirEstado();
  if (proteccionActiva) {
    Serial.printf("Tension servos %.2f V: PROTECCION ACTIVA (critica < %.1f V -> HOME, corte < %.1f V -> sueltos)\n",
                  tension, TENSION_CRITICA, TENSION_CORTE);
  } else {
    Serial.printf("ATENCION: no se mide tension en los servos (%.2f V en GPIO%d): fuente apagada o sensor desconectado.\n"
                  "          La proteccion se activa sola cuando detecte mas de %.1f V.\n",
                  tension, PIN_TENSION, TENSION_RECUPERADA);
  }
  Serial.println("\nServos SUELTOS. Mandar h para activarlos (van directo a HOME = mano extendida, muneca derecha: los dos DS3225 en 100).");
}

void loop() {
  leerSerie();
  actualizarSecuencia();
  actualizarServos();
  revisarTension();
  revisarPCA();
}
