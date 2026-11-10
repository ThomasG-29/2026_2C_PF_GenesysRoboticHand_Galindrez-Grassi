// =====================================================
// PAQUETE ESP-NOW GUANTE -> MANO
// IMPORTANTE: este archivo esta COPIADO en los dos proyectos
//   5. Guante - ESP32-C3(Transmisor)/src/paquete.h
//   5. Mano - ESP32(Receptor)/src/paquete.h
// Si se cambia uno, copiarlo igual en el otro. Si no coinciden,
// el receptor descarta los paquetes y avisa "paquete de largo distinto".
// =====================================================
#pragma once
#include <stdint.h>

// Canal WiFi de ESP-NOW (igual en las dos placas)
#define CANAL_ESPNOW 1

#define TIPO_GUANTE 5

// Cantidad de sensores flex del guante: 1-5 dedos, 6 muneca adelante,
// 7 y 8 costados de la muneca (el 8, del lado del pulgar, no tiene servo:
// el receptor lo usa para separar el costado del movimiento hacia adelante).
#define CANT_FLEX 8

// Potenciometros del mecanismo de cada dedo (2 por dedo), en este orden:
//   pot[0..4] base  (apertura lateral): pulgar, indice, medio, anular, menique
//   pot[5..9] arriba (flexion del nudillo): pulgar, indice, medio, anular, menique
// El dedo medio se manda igual aunque por ahora la mano no lo use.
#define CANT_POT 10
#define POT_BASE   0   // primer potenciometro de la base
#define POT_ARRIBA 5   // primer potenciometro de arriba

// Cada cuantos paquetes se imprime una linea de historial.
// Los dos lados imprimen los MISMOS numeros de secuencia
// (multiplos de este valor) para poder comparar linea a linea.
#define HISTORIAL_CADA_N 5

typedef struct __attribute__((packed)) {
  uint8_t  tipo;                     // TIPO_GUANTE
  uint32_t seq;                      // numero de secuencia (detecta paquetes perdidos)
  uint32_t t_ms;                     // millis() del transmisor al medir
  int16_t  adc[CANT_FLEX];           // lecturas del ADS1115 (filtradas con mediana)
  uint32_t resistencia[CANT_FLEX];   // resistencia de cada flex en ohms
  int16_t  pot[CANT_POT];            // lectura de cada potenciometro (cuentas del ADS, 3.3 V = ~26400)
  uint16_t bateria_mV;               // tension de la bateria del guante (0 = no se puede medir)
  uint8_t  bateriaBaja;              // 1: bateria descargada -> la mano va a HOME y no sigue
                                     // al guante hasta que se cargue (lo decide el transmisor)
  uint8_t  estadoCarga;              // CARGA_NO / CARGA_CARGANDO / CARGA_LLENA (deducido de la
                                     // tension: ver CARGA en el transmisor)
  // Botones del guante (se manda el ESTADO en cada paquete: si se pierde uno no pasa nada)
  uint8_t  paradaEmergencia;         // 1: boton de parada apretado (con retencion): servos quietos
  uint8_t  home;                     // 1: HOME pedido con el boton (se apreta otra vez para salir)
  uint8_t  pedidosCal;               // sube 1 cada vez que se aprieta el boton de calibracion
                                     // (el receptor calibra cuando cambia, igual que con c)
} PaqueteGuante;

// Estado del cargador de la bateria del guante
enum { CARGA_NO = 0, CARGA_CARGANDO = 1, CARGA_LLENA = 2 };

// =====================================================
// PAQUETE DE VUELTA MANO -> GUANTE (para el LED de calibracion)
// El receptor lo manda cada 100 ms (y en cuanto empieza o termina una
// calibracion) a la MAC de la que le llegan los paquetes del guante.
// =====================================================
#define TIPO_MANO 6

typedef struct __attribute__((packed)) {
  uint8_t tipo;         // TIPO_MANO
  uint8_t calibrando;   // 1: hay una calibracion en curso (boton, c o m)
  uint8_t patronLed;    // que hacer en la fase actual (patron del LED de calibracion)
} PaqueteMano;

// Patron del LED de calibracion segun lo que hay que hacer en cada fase
enum {
  LED_CAL_NO = 0,       // sin calibracion
  LED_CAL_QUIETA,       // parpadeo LENTO: mano abierta, muneca derecha, quieta
  LED_CAL_MOVER,        // parpadeo RAPIDO: hacer el movimiento que sigue (cerrar la mano,
                        //   muneca adelante / al menique / al pulgar)
  LED_CAL_SEPARAR,      // DOBLE DESTELLO: abrir y separar los dedos
};
