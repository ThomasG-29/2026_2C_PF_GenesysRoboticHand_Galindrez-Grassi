// Prueba del diferencial de la muñeca - Arduino Uno
// 2 servos GXservo Y3225MG (25 kg): servo A en pin 3, servo B en pin 5
//
// Al encender, los dos servos van a 90° (centro) para ensamblar.
//
// Monitor serie del Arduino IDE a 9600 baudios. Comandos:
//   M      -> modo M (mismo sentido): los dos servos al mismo ángulo -> abducción
//   I      -> modo I (invertido): servo B al ángulo opuesto -> giro de la muñeca
//   120    -> mueve en el modo actual
//   M120   -> cambia a modo M y mueve (también "M 120")
//   I60    -> cambia a modo I y mueve
//   90     -> en cualquier modo vuelve los dos servos al centro
//
//   Modo M, ángulo X:  A = X,  B = X          (ej. M120 -> A=120, B=120)
//   Modo I, ángulo X:  A = X,  B = 180 - X    (ej. I120 -> A=120, B=60)
//
// Cada comando es una posición absoluta medida desde el centro (90°):
// no se suman las abducciones y los giros anteriores.
//
// MOVIMIENTO SINCRONIZADO: los dos servos no saltan al ángulo final (cada uno
// iría a su velocidad máxima, uno llega antes que el otro y en el medio la
// muñeca mezcla giro con abducción). Se mueven juntos en pasos chicos, con
// arranque y frenado suaves, y llegan al mismo tiempo. La rapidez se ajusta
// con velocidadMedia.
//
// Si al probar el modo M la muñeca gira en vez de hacer abducción (porque los
// servos están montados en espejo), poner invertirB = true.
//
// ALIMENTACIÓN: dos servos de 25 kg pueden consumir 5 A o más juntos.
// Alimentarlos con una fuente externa (5-6 V, 5 A o más) y unir el GND de
// la fuente con el GND del Arduino. Por USB la placa se reinicia.

#include <Servo.h>

Servo servoA;
Servo servoB;

const int pinServoA = 3;
const int pinServoB = 5;
const int anguloInicial = 90;      // Posición de ensamblado

const bool invertirB = false;      // true si el servo B está montado en espejo

// Rango de pulso del servo: 500 us = 0°, 1500 us = 90° (centro), 2500 us = 180°
const int pulsoMin = 500;
const int pulsoMax = 2500;

// Movimiento sincronizado
const float velocidadMedia = 60.0; // grados por segundo del servo que más recorre
const int pasoMs = 20;             // un paso por cada pulso del servo (50 Hz)

char modo = 'M';                   // 'M' = mismo sentido, 'I' = invertido
float actualA = anguloInicial;     // último ángulo enviado a cada servo
float actualB = anguloInicial;

// En microsegundos: write() solo acepta grados enteros y el paso de 1° se nota
int pulso(float angulo) {
  return pulsoMin + angulo * (pulsoMax - pulsoMin) / 180.0;
}

void escribirA(float angulo) {
  servoA.writeMicroseconds(pulso(angulo));
}

// Escribe el ángulo del servo B teniendo en cuenta si está montado en espejo
void escribirB(float angulo) {
  servoB.writeMicroseconds(pulso(invertirB ? 180 - angulo : angulo));
}

// Lleva los dos servos a su destino al mismo tiempo: en cada paso los dos
// avanzan la misma fracción de su recorrido, así que el diferencial hace un
// solo movimiento (giro o abducción) sin mezclar el otro.
void moverJuntos(float destinoA, float destinoB) {
  float inicioA = actualA, inicioB = actualB;
  float difA = destinoA - inicioA, difB = destinoB - inicioB;
  float mayor = max(fabs(difA), fabs(difB));
  unsigned long duracion = mayor / velocidadMedia * 1000;

  unsigned long t0 = millis();
  while (millis() - t0 < duracion) {
    float t = (float)(millis() - t0) / duracion;
    float s = 0.5 - 0.5 * cos(PI * t);   // arranca y frena despacio (sin tirón)
    escribirA(inicioA + difA * s);
    escribirB(inicioB + difB * s);
    delay(pasoMs);
  }
  escribirA(destinoA);
  escribirB(destinoB);
  actualA = destinoA;
  actualB = destinoB;
}

void mostrarModo() {
  Serial.print("Modo ");
  Serial.print(modo);
  Serial.println(modo == 'M' ? " (mismo sentido -> abduccion)" : " (invertido -> giro de muneca)");
}

void mover(int angulo) {
  int anguloA = angulo;
  int anguloB = (modo == 'M') ? angulo : 180 - angulo;

  Serial.print("-> Modo ");
  Serial.print(modo);
  Serial.print(" | A (pin 3) = ");
  Serial.print(anguloA);
  Serial.print("  B (pin 5) = ");
  Serial.println(anguloB);

  moverJuntos(anguloA, anguloB);
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);  // Funciona con o sin fin de línea en el monitor
  // Si este mensaje se repite solo, la placa se reinicia al conectar los servos
  // (cae la tensión): revisar la alimentación de los servos
  Serial.println("\nArrancando...");
  Serial.flush();

  // Se manda la posición antes de attach() para que el primer pulso ya sea 90°
  escribirA(anguloInicial);
  escribirB(anguloInicial);
  servoA.attach(pinServoA, pulsoMin, pulsoMax);
  servoB.attach(pinServoB, pulsoMin, pulsoMax);

  Serial.println("Diferencial de la muneca listo. A y B en 90 grados.");
  mostrarModo();
  Serial.println("Comandos: M, I, un angulo (0-180), o M120 / I60");
}

void loop() {
  if (!Serial.available()) return;

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();
  entrada.toUpperCase();
  if (entrada.length() == 0) return;

  // Cambio de modo con la letra del principio
  char letra = entrada.charAt(0);
  if (letra == 'M' || letra == 'I') {
    modo = letra;
    entrada = entrada.substring(1);
    entrada.trim();
    if (entrada.length() == 0) {
      mostrarModo();
      return;
    }
  }

  long valor = entrada.toInt();
  if (valor == 0 && entrada != "0") {
    Serial.print("Comando no valido: ");
    Serial.println(entrada);
    return;
  }

  if (valor < 0 || valor > 180) {
    Serial.print("Fuera de rango: ");
    Serial.print(valor);
    Serial.println("  (angulo 0-180)");
    return;
  }

  mover((int)valor);
}
