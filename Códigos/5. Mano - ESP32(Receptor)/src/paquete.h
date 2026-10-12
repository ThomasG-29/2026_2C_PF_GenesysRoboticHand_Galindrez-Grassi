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

// Cantidad de sensores flex del guante.
// (Los 10 potenciometros se agregan mas adelante: sumar aca un
//  arreglo pot[CANT_POT] al final del paquete, en los dos archivos.)
#define CANT_FLEX 7

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
} PaqueteGuante;
