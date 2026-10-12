// =====================================================
// PAQUETE ESP-NOW COMPARTIDO POR TRANSMISOR Y RECEPTOR
// (un solo archivo: los dos lados siempre usan la misma estructura)
// =====================================================
#pragma once
#include <stdint.h>

// Canal WiFi de ESP-NOW (igual en las dos placas)
#define CANAL_ESPNOW 1

#define TIPO_FLEX 3

// Cada cuantos paquetes se imprime una linea de historial.
// Los dos lados imprimen los MISMOS numeros de secuencia
// (multiplos de este valor) para poder comparar linea a linea.
#define HISTORIAL_CADA_N 5

typedef struct __attribute__((packed)) {
  uint8_t  tipo;          // TIPO_FLEX
  uint32_t seq;           // numero de secuencia (detecta paquetes perdidos)
  uint32_t t_ms;          // millis() del transmisor al medir
  int16_t  adc;           // lectura del ADS1115 (filtrada con mediana)
  uint32_t resistencia;   // resistencia del flex 1 en ohms
} PaqueteFlex;
