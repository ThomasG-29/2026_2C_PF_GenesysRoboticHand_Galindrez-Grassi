// =====================================================
// TRANSMISOR - ESP32-C3 MINI (guante, COM5)
// Lee los 8 sensores flex y los 10 potenciometros de los dedos
// (2 multiplexores -> ADS1115), calcula la resistencia de cada flex y
// manda todo por ESP-NOW al receptor (mano robot).
//
// Conexionado igual que Conexionado_guante-v3:
//   Flex 1           -> multiplexor 1, canal 0      -> A0 del ADS1115
//                       (antes en mux2 C0; se movio al mux1 cuando se cambiaron los dos mux)
//   Flex 2, 3, 5-7   -> multiplexor 1, canales 1, 2, 4-6 -> A0 del ADS1115
//   Flex 4           -> multiplexor 1, canal 13     -> A0 (el C3 tira la senal a GND)
//   Flex 8           -> multiplexor 1, canal 12     -> A0 (costado del pulgar de la muneca)
//   Pot. base (x5)   -> multiplexor 1, canales 7-11 -> A0 (pulgar ... menique)
//   Pot. arriba (x5) -> multiplexor 2, canales 1-5  -> A1 (pulgar ... menique)
//
// El conversor mide de a 16 cuentas (~2 mV): probablemente es un ADS1015
// (12 bits) vendido como ADS1115. Un ADS1115 original mide 16 veces mas fino.
//
// Flex 6, 7 y 8 (muneca): se arrastran entre si por como estan en el guante.
// Aca no se corrige nada, se manda la resistencia tal cual y el receptor
// separa el movimiento hacia adelante del movimiento hacia los costados.
//
// BATERIA (ver BATERIA mas abajo): divisor de 10k/10k en A2 del ADS1115.
// Por debajo de BATERIA_BAJA avisa a la mano (bateriaBaja = 1 en el paquete):
// la mano va a HOME y no sigue al guante hasta que la bateria se cargue
// (pase de BATERIA_CARGADA). El bloqueo queda guardado: apagar y prender
// el guante no lo saca, solo cargarlo.
// Comando por el monitor serie: b 3.95 = corregir la medicion con lo que
// marca el tester en la bateria (se guarda en la memoria).
// CARGA: sin cables extra, se deduce si el cargador esta enchufado por como
// cambia la tension (ver CARGA mas abajo) y se manda a la mano (estadoCarga).
// =====================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Preferences.h>
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
// S0-S3 se comparten entre los dos multiplexores:
// seleccionan el mismo canal en los dos al mismo tiempo.

#define ADS_CANAL_MUX1 0   // SIG del multiplexor 1 -> A0 del ADS1115
#define ADS_CANAL_MUX2 1   // SIG del multiplexor 2 -> A1 del ADS1115

// Donde esta cada flex: canal del multiplexor y entrada del ADS1115
typedef struct {
  uint8_t canalMux;
  uint8_t entradaAds;
} SensorFlex;

const SensorFlex FLEX[CANT_FLEX] = {
  {0, ADS_CANAL_MUX1},   // Flex 1
  {1, ADS_CANAL_MUX1},   // Flex 2
  {2, ADS_CANAL_MUX1},   // Flex 3
  {13, ADS_CANAL_MUX1},  // Flex 4 (el C3 de este mux tira la senal a GND)
  {4, ADS_CANAL_MUX1},   // Flex 5
  {5, ADS_CANAL_MUX1},   // Flex 6
  {6, ADS_CANAL_MUX1},   // Flex 7
  {12, ADS_CANAL_MUX1},  // Flex 8 (costado del pulgar de la muneca)
};

// =====================================================
// POTENCIOMETROS DEL MECANISMO DE CADA DEDO (Mecanismo de sensorizacion.iam)
// Dos por dedo, en el orden de pot[] del paquete:
//   base (apertura lateral) -> multiplexor 1, C7 pulgar ... C11 menique
//   arriba (flexion nudillo) -> multiplexor 2, C1 pulgar ... C5 menique
// (verificado moviendo cada dedo el 2026-10-24; M2 C6 no se usa)
// =====================================================
const SensorFlex POT[CANT_POT] = {
  {7,  ADS_CANAL_MUX1},   // base pulgar
  {8,  ADS_CANAL_MUX1},   // base indice
  {9,  ADS_CANAL_MUX1},   // base medio
  {10, ADS_CANAL_MUX1},   // base anular
  {11, ADS_CANAL_MUX1},   // base menique
  {1,  ADS_CANAL_MUX2},   // arriba pulgar
  {2,  ADS_CANAL_MUX2},   // arriba indice
  {3,  ADS_CANAL_MUX2},   // arriba medio
  {4,  ADS_CANAL_MUX2},   // arriba anular
  {5,  ADS_CANAL_MUX2},   // arriba menique
};
const char *NOMBRE_POT[CANT_POT] = {"Bpul", "Bind", "Bmed", "Banu", "Bmen",
                                    "Apul", "Aind", "Amed", "Aanu", "Amen"};

Adafruit_ADS1115 ads;

// =====================================================
// DIVISOR DE TENSION DEL FLEX
// La resistencia fija va entre VCC y la salida y el flex entre la salida
// y GND (Conexionados/Circuito_guante - v2_bb): V = VCC * R / (R_FIJA + R).
// Como estan montados en el guante, al doblarlo BAJA su resistencia y la lectura.
// =====================================================
#define VCC_DIVISOR 3.3f       // alimentacion del divisor (V)
#define R_FIJA      10000.0f   // resistencia fija del divisor (ohms)

// Lecturas por sensor en cada envio (la mediana descarta picos).
// 8 flex x 3 lecturas + 10 potenciometros x 1 = ~31 ms (entra en los 40 ms).
#define MUESTRAS_MEDIANA 3

// =====================================================
// BATERIA (LiPo 3.7 V 1000 mAh, Conexionados/Circuito_guante - v2_bb)
// B+ -> 10k -> A2 del ADS1115 -> 10k -> GND: A2 = bateria / 2
// (bateria llena 4.2 V -> 2.1 V en A2, dentro del rango del ADS a 3.3 V)
// =====================================================
#define ADS_CANAL_BATERIA     2
#define DIVISOR_BATERIA       ((10000.0f + 10000.0f) / 10000.0f)
#define BATERIA_AVISO         3.60f   // V: solo avisa por el monitor (queda poca carga)
#define BATERIA_BAJA          3.30f   // V: la mano va a HOME y se bloquea hasta cargar
                                      //    (queda ~5 % de carga: margen para que el guante siga
                                      //    mandando paquetes y la mano se quede en HOME)
#define BATERIA_CARGADA       3.90f   // V: se desbloquea (hace falta enchufar el cargador:
                                      //    la bateria sola no vuelve a subir tanto)
#define BATERIA_SIN_MEDICION  2.50f   // V: por debajo, el divisor no esta conectado
                                      //    (una LiPo nunca llega tan abajo: el TP4056 la corta)
#define TIEMPO_BATERIA_MS     5000    // tiene que estar baja (o cargada) este tiempo seguido
#define PERIODO_BATERIA_MS    200     // cada cuanto se mide (una lectura del ADS, ~1.2 ms)

Preferences memoria;
float bateria = 0;              // tension de la bateria (V, filtrada)
float factorBateria = 1.0f;     // correccion por la tolerancia de las resistencias (comando b)
bool bateriaBaja = false;       // bloqueo: se guarda en la memoria hasta que se carga
bool avisoBateria = false;

// =====================================================
// CARGA (deducida de la tension, sin conectar nada mas)
// Al enchufar el cargador la tension pega un salto para arriba (la corriente
// de carga por la resistencia interna de la bateria); al desenchufarlo, el salto
// es para abajo. Se compara el promedio de los ultimos 3 s con el de hace 10 s.
// (Una subida LENTA no sirve: la lectura oscila +-15 mV por el consumo a rafagas
// del WiFi y la bateria se recupera sola unos mV cuando baja el consumo; con eso
// el 2026-11-08 marcaba "cargando" sin cargador. Cargando de verdad sube solo
// ~10-15 mV por minuto.)
// Limitaciones: con la bateria casi llena la tension casi no cambia, y si el
// cargador ya estaba enchufado al prender el guante no hay salto que ver
// (enchufarlo con el guante ya prendido).
// =====================================================
#define CARGA_SALTO_V          0.06f  // V: salto (enchufar / desenchufar). Ruido medido: +-0.017
#define CARGA_PROMEDIO_S       3      // s: muestras que se promedian de cada lado
#define CARGA_SEPARACION_S     10     // s: entre el promedio de ahora y el de antes
#define BATERIA_LLENA          4.15f  // V: cargando y por encima de esto = llena (o casi)
#define BATERIA_LLENA_SALIDA   4.08f  // V: estando llena, por debajo = cargador desenchufado
#define CARGA_HISTORIAL_S      (CARGA_SEPARACION_S + CARGA_PROMEDIO_S)
#define CARGA_ESPERA_INICIO_MS 15000  // al encender la lectura sube ~0.08 V en 5 s (arranque del
                                      // WiFi): recien despues de esto se empieza a mirar
#define CARGA_ESPERA_USB_MS    60000  // al conectar/desconectar la PC con el boton prendido la C3
                                      // toma parte de la energia del USB, la bateria queda con menos
                                      // consumo y su tension sube como si cargara (de golpe y despues
                                      // despacio ~1 min): se espera a que se estabilice

uint8_t estadoCarga = CARGA_NO;
float historialBateria[CARGA_HISTORIAL_S];   // una muestra por segundo
int histPos = 0, histCant = 0;
float ultimoSalto = 0;          // ultimo salto medido (se ve con el comando b)
uint32_t tCambioUsb = 0;        // cuando se conecto/desconecto la PC (0 = nunca)

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
int16_t leerFlex(const SensorFlex &s) {
  selectMuxChannel(s.canalMux);
  int16_t m[MUESTRAS_MEDIANA];
  for (int i = 0; i < MUESTRAS_MEDIANA; i++) {
    int16_t v = ads.readADC_SingleEnded(s.entradaAds);
    int j = i;
    while (j > 0 && m[j - 1] > v) { m[j] = m[j - 1]; j--; }  // insercion ordenada
    m[j] = v;
  }
  return m[MUESTRAS_MEDIANA / 2];
}

uint32_t calcularResistencia(int16_t adc) {
  float v = ads.computeVolts(adc);
  if (v >= VCC_DIVISOR - 0.001f) return 9999999;   // salida en VCC: flex abierto (sin GND)
  if (v <= 0) return 0;                            // salida en GND: flex en corto
  return (uint32_t)(R_FIJA * v / (VCC_DIVISOR - v));
}

// Un potenciometro es un divisor de baja impedancia: con una lectura alcanza
int16_t leerPot(const SensorFlex &s) {
  selectMuxChannel(s.canalMux);
  return ads.readADC_SingleEnded(s.entradaAds);
}

// =====================================================
// BATERIA
// =====================================================
float leerBateria() {
  return ads.computeVolts(ads.readADC_SingleEnded(ADS_CANAL_BATERIA)) * DIVISOR_BATERIA * factorBateria;
}

bool bateriaMedible() { return bateria >= BATERIA_SIN_MEDICION; }

void guardarBloqueo(bool baja) {
  bateriaBaja = baja;
  memoria.putBool("batBaja", baja);
}

// Promedio de CARGA_PROMEDIO_S muestras, empezando por la de hace "desde" segundos (0 = la ultima)
float promedioBateria(int desde) {
  float suma = 0;
  for (int s = desde; s < desde + CARGA_PROMEDIO_S; s++)
    suma += historialBateria[(histPos - 1 - s + 2 * CARGA_HISTORIAL_S) % CARGA_HISTORIAL_S];
  return suma / CARGA_PROMEDIO_S;
}

void reiniciarHistorial() { histPos = histCant = 0; }

const char *textoCarga(uint8_t e) {
  return e == CARGA_CARGANDO ? "CARGANDO" : e == CARGA_LLENA ? "LLENA (cargador enchufado)" : "no carga";
}

void cambiarCarga(uint8_t nuevo) {
  if (nuevo == estadoCarga) return;
  estadoCarga = nuevo;
  reiniciarHistorial();   // el salto que se acaba de ver no tiene que volver a contar
  Serial.printf(">>> CARGA: %s (%.2f V)", textoCarga(nuevo), bateria);
  if (nuevo == CARGA_CARGANDO && bateriaBaja)
    Serial.printf(" | la mano se desbloquea al pasar %.2f V", BATERIA_CARGADA);
  if (nuevo == CARGA_NO) Serial.print(" | cargador desenchufado");
  Serial.println();
}

// Se llama una vez por segundo con la tension filtrada
void revisarCarga() {
  if (millis() < CARGA_ESPERA_INICIO_MS) return;

  // Conectar o desconectar la PC por USB cambia de donde se alimenta la C3 (y el
  // consumo de la bateria): no se mira hasta que la tension se estabiliza
  static bool usbAntes = Serial.isPlugged();
  if (Serial.isPlugged() != usbAntes) {
    usbAntes = !usbAntes;
    tCambioUsb = millis();
    Serial.printf(">>> CARGA: PC %s, la deteccion espera %d s a que se estabilice la bateria\n",
                  usbAntes ? "conectada" : "desconectada", CARGA_ESPERA_USB_MS / 1000);
  }
  if (tCambioUsb && millis() - tCambioUsb < CARGA_ESPERA_USB_MS) {
    reiniciarHistorial();
    return;
  }

  historialBateria[histPos] = bateria;
  histPos = (histPos + 1) % CARGA_HISTORIAL_S;
  if (histCant < CARGA_HISTORIAL_S) histCant++;
  if (histCant < CARGA_HISTORIAL_S) return;

  float salto = promedioBateria(0) - promedioBateria(CARGA_SEPARACION_S);
  ultimoSalto = salto;

  if (estadoCarga == CARGA_NO) {
    if (salto >= CARGA_SALTO_V) cambiarCarga(bateria >= BATERIA_LLENA ? CARGA_LLENA : CARGA_CARGANDO);
  } else if (estadoCarga == CARGA_CARGANDO) {
    if (salto <= -CARGA_SALTO_V) cambiarCarga(CARGA_NO);
    else if (bateria >= BATERIA_LLENA) cambiarCarga(CARGA_LLENA);
  } else {   // CARGA_LLENA: la tension queda casi fija, solo se mira si baja
    if (salto <= -CARGA_SALTO_V || bateria < BATERIA_LLENA_SALIDA) cambiarCarga(CARGA_NO);
  }
}

// Se llama en cada vuelta de loop(): mide cada PERIODO_BATERIA_MS y decide el bloqueo
void revisarBateria() {
  static uint32_t tMedicion = 0, tBaja = 0, tCargada = 0, tAvisoSinMedicion = 0, tCarga = 0;
  uint32_t ahora = millis();
  if (ahora - tMedicion < PERIODO_BATERIA_MS) return;
  tMedicion = ahora;

  float v = leerBateria();
  bateria = bateria == 0 ? v : bateria + 0.2f * (v - bateria);   // filtro: ~1 s

  if (!bateriaMedible()) {
    tBaja = tCargada = 0;
    estadoCarga = CARGA_NO;
    reiniciarHistorial();
    if (ahora - tAvisoSinMedicion > 10000) {
      tAvisoSinMedicion = ahora;
      Serial.printf(">>> BATERIA: no se puede medir (A2 = %.2f V): revisar el divisor. PROTECCION DESACTIVADA\n",
                    bateria / DIVISOR_BATERIA / factorBateria);
    }
    return;
  }

  if (ahora - tCarga >= 1000) {
    tCarga = ahora;
    revisarCarga();
  }

  if (!bateriaBaja) {
    if (bateria < BATERIA_BAJA) {
      if (!tBaja) tBaja = ahora;
      if (ahora - tBaja >= TIEMPO_BATERIA_MS) {
        guardarBloqueo(true);
        Serial.printf(">>> BATERIA BAJA: %.2f V (< %.2f V). La mano va a HOME y queda bloqueada hasta cargar el guante.\n",
                      bateria, BATERIA_BAJA);
      }
    } else {
      tBaja = 0;
    }
  } else {
    if (bateria >= BATERIA_CARGADA) {
      if (!tCargada) tCargada = ahora;
      if (ahora - tCargada >= TIEMPO_BATERIA_MS) {
        guardarBloqueo(false);
        Serial.printf(">>> BATERIA CARGADA: %.2f V. La mano vuelve a seguir al guante.\n", bateria);
      }
    } else {
      tCargada = 0;
    }
  }

  if (!avisoBateria && bateria < BATERIA_AVISO && ahora > CARGA_ESPERA_INICIO_MS) {
    avisoBateria = true;
    Serial.printf(">>> BATERIA: %.2f V, queda poca carga (bloqueo a %.2f V)\n", bateria, BATERIA_BAJA);
  } else if (avisoBateria && bateria > BATERIA_AVISO + 0.1f) {
    avisoBateria = false;
  }
}

// Comando b: muestra la bateria; "b 3.95" corrige la medicion con lo que marca
// el tester entre B+ y B- (se guarda en la memoria)
void comandoBateria(const char *args) {
  float real = atof(args);
  if (real > 0) {
    float medida = 0;
    for (int k = 0; k < 10; k++) medida += leerBateria();
    medida /= 10;
    if (medida < BATERIA_SIN_MEDICION) {
      Serial.println(">>> BATERIA: no se puede medir, no se corrige");
      return;
    }
    factorBateria *= real / medida;
    memoria.putFloat("factorB", factorBateria);
    bateria = real;
    Serial.printf(">>> BATERIA: corregida (factor %.3f) y guardada\n", factorBateria);
  }
  Serial.printf(">>> BATERIA: %.3f V | aviso < %.2f | bloqueo < %.2f | desbloqueo > %.2f | %s | %s",
                bateria, BATERIA_AVISO, BATERIA_BAJA, BATERIA_CARGADA,
                !bateriaMedible() ? "NO SE PUEDE MEDIR" : bateriaBaja ? "BLOQUEADA (cargar)" : "OK",
                textoCarga(estadoCarga));
  uint32_t ahora = millis();
  if (ahora < CARGA_ESPERA_INICIO_MS)
    Serial.println(" (todavia no mira: recien encendido)");
  else if (tCambioUsb && ahora - tCambioUsb < CARGA_ESPERA_USB_MS)
    Serial.printf(" (todavia no mira: faltan %lu s por el USB)\n",
                  (unsigned long)((CARGA_ESPERA_USB_MS - (ahora - tCambioUsb)) / 1000));
  else
    Serial.printf(" (salto %+.3f V; cargando si pasa %+.2f)\n", ultimoSalto, CARGA_SALTO_V);
}

void leerComandos() {
  static char linea[32];
  static uint8_t largo = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (!largo) continue;
      linea[largo] = 0;
      largo = 0;
      if (tolower(linea[0]) == 'b') comandoBateria(linea + 1);
      else Serial.println(">>> Comandos: b = ver bateria | b 3.95 = corregir con lo que marca el tester");
    } else if (largo < sizeof(linea) - 1) {
      linea[largo++] = c;
    }
  }
}

void imprimirEncabezado() {
  Serial.print("ENVIADO     tiempo     seq |");
  for (int i = 0; i < CANT_FLEX; i++) Serial.printf("    R%d", i + 1);
  Serial.print(" (ohm) |");
  for (int i = 0; i < CANT_POT; i++) Serial.printf(" %5s", NOMBRE_POT[i]);
  Serial.println(" | lect.  ACK ok | bateria");
}

void setup() {
  Serial.begin(115200);
  // USB nativo: sin monitor abierto, imprimir espera como maximo 1 ms y
  // descarta el texto (con el valor por defecto la salida se traba tras un
  // rato; con 0 la libreria se queda esperando para siempre)
  Serial.setTxTimeoutMs(1);
  delay(1500);

  Serial.println("\n=== GUANTE: 8 FLEX + 10 POTENCIOMETROS -> ESP-NOW (ESP32-C3) ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);                     // I2C rapido: 21 lecturas por envio
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

  // Bateria: el bloqueo se recuerda aunque se apague el guante
  memoria.begin("guante", false);
  factorBateria = memoria.getFloat("factorB", 1.0f);
  bateriaBaja = memoria.getBool("batBaja", false);
  for (int k = 0; k < 10; k++) bateria += leerBateria();
  bateria /= 10;
  if (bateriaMedible())
    Serial.printf("Bateria: %.2f V (bloqueo < %.2f V, desbloqueo > %.2f V)%s\n", bateria, BATERIA_BAJA,
                  BATERIA_CARGADA, bateriaBaja ? " | BLOQUEADA: cargar el guante" : "");
  else
    Serial.printf("ATENCION: no se puede medir la bateria (A2 = %.2f V): PROTECCION DESACTIVADA\n",
                  bateria / DIVISOR_BATERIA / factorBateria);

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
  imprimirEncabezado();
}

void loop() {
  static uint32_t proximoEnvio = millis();
  static PaqueteGuante p = {};

  leerComandos();
  if ((int32_t)(millis() - proximoEnvio) < 0) return;
  proximoEnvio += PERIODO_ENVIO_MS;
  // Si el loop se atraso, se resincroniza en vez de mandar una rafaga de paquetes
  if ((int32_t)(millis() - proximoEnvio) >= 0) proximoEnvio = millis() + PERIODO_ENVIO_MS;

  uint32_t inicioLectura = millis();
  for (int i = 0; i < CANT_FLEX; i++) {
    p.adc[i] = leerFlex(FLEX[i]);
    p.resistencia[i] = calcularResistencia(p.adc[i]);
  }
  for (int i = 0; i < CANT_POT; i++) p.pot[i] = leerPot(POT[i]);
  revisarBateria();
  uint32_t tiempoLectura = millis() - inicioLectura;

  // Sin medicion de bateria no se bloquea la mano (el divisor podria estar suelto)
  p.bateria_mV = bateriaMedible() ? (uint16_t)lroundf(bateria * 1000) : 0;
  p.bateriaBaja = bateriaMedible() && bateriaBaja;
  p.estadoCarga = estadoCarga;
  p.tipo = TIPO_GUANTE;
  p.seq++;
  p.t_ms = inicioLectura;
  esp_now_send(macReceptor, (uint8_t *)&p, sizeof(p));

  if (p.seq % HISTORIAL_CADA_N == 0) {
    if (p.seq % (HISTORIAL_CADA_N * 40) == 0) imprimirEncabezado();
    uint32_t ok = ackOk, fail = ackFallidos;
    Serial.printf("ENVIADO  %8.2f s  #%6lu |", p.t_ms / 1000.0, (unsigned long)p.seq);
    for (int i = 0; i < CANT_FLEX; i++) Serial.printf(" %5lu", (unsigned long)p.resistencia[i]);
    Serial.print("       |");
    for (int i = 0; i < CANT_POT; i++) Serial.printf(" %5d", p.pot[i]);
    Serial.printf(" | %2lu ms  %5.1f%% | %4.2f V%s%s\n", (unsigned long)tiempoLectura,
                  ok + fail ? 100.0 * ok / (ok + fail) : 0.0, bateria,
                  p.bateriaBaja ? " BAJA" : "",
                  estadoCarga == CARGA_CARGANDO ? " cargando" : estadoCarga == CARGA_LLENA ? " llena" : "");
  }
}
