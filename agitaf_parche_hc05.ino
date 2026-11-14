// AGITAF - Parche con MPU6050 + HC-05 (Bluetooth clásico)
// Placa: Arduino Uno o Nano (5 V)
//
// CABLEADO
//   MPU6050 (GY-521): VCC -> 5V | GND -> GND | SDA -> A4 | SCL -> A5
//   HC-05:            VCC -> 5V | GND -> GND
//                     TXD -> D10 (el Arduino recibe)
//                     RXD <- D11 a través de divisor de voltaje:
//                            D11 -> resistencia 1 kΩ -> RXD del HC-05
//                            RXD del HC-05 -> resistencia 2 kΩ -> GND
//   (el RXD del HC-05 es de 3,3 V; conectarlo directo a 5 V puede dañarlo)
//
// El HC-05 viene a 9600 baudios de fábrica: no hace falta configurarlo.
// La página se conecta por el puerto serie que crea el computador al emparejarlo.

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <math.h>

Adafruit_MPU6050 mpu;
SoftwareSerial bt(10, 11);              // RX = D10, TX = D11
Print *salidas[2] = { &Serial, &bt };   // los mensajes salen por USB y por Bluetooth

// CONFIGURACION
const float UMBRAL_G = 2.2;
const unsigned long INTERVALO_MS = 10;
const unsigned long VENTANA_EVENTO_MS = 200;
const unsigned long ESPERA_EVENTO_MS = 450;
const unsigned long INTERVALO_BT_MS = 100;   // datos por Bluetooth: 10 por segundo

// CONTROL DE SESION
bool pausado = true;
bool registrando = false;
bool primerEvento = true;

unsigned long ultimaLectura = 0;
unsigned long ultimoReporte = 0;
unsigned long ultimoBT = 0;
unsigned long inicioEvento = 0;
unsigned long ultimoEvento = 0;

unsigned int contador = 0;
float picoEvento = 0;
float picoGiroEvento = 0;
float sumaPicos = 0;
float picoMaximoSesion = 0;

// Mensaje informativo por USB y Bluetooth (las líneas que parten con # las ignora el algoritmo)
void mensaje(const __FlashStringHelper *texto) {
  for (byte i = 0; i < 2; i++) {
    salidas[i]->println(texto);
  }
}

void reiniciarSesion() {
  pausado = true;
  registrando = false;
  primerEvento = true;
  contador = 0;
  picoEvento = 0;
  picoGiroEvento = 0;
  sumaPicos = 0;
  picoMaximoSesion = 0;
  ultimoEvento = 0;

  mensaje(F("# SESION REINICIADA"));
  mensaje(F("# Escribe R para comenzar"));
}

void resumen() {
  for (byte i = 0; i < 2; i++) {
    Print *p = salidas[i];
    p->println(F("# RESUMEN DE SESION"));
    p->print(F("# Eventos: "));
    p->println(contador);
    p->print(F("# Pico maximo: "));
    p->print(picoMaximoSesion, 2);
    p->println(F(" g"));
    if (contador > 0) {
      p->print(F("# Promedio de picos: "));
      p->print(sumaPicos / contador, 2);
      p->println(F(" g"));
    }
  }
}

void procesarComando(char comando) {
  if (comando == 'P' || comando == 'p') {
    pausado = true;
    registrando = false;
    mensaje(F("# PAUSADO"));
  }

  if (comando == 'R' || comando == 'r') {
    pausado = false;
    registrando = false;
    ultimaLectura = millis();
    ultimoReporte = millis();
    ultimoBT = millis();
    mensaje(F("# MEDICIONES ACTIVADAS"));
  }

  if (comando == 'C' || comando == 'c') {
    reiniciarSesion();
  }

  if (comando == 'S' || comando == 's') {
    resumen();
  }
}

// Comandos desde el monitor serie O desde la página web por Bluetooth
void controlarComandos() {
  while (Serial.available() > 0) {
    procesarComando(Serial.read());
  }
  while (bt.available() > 0) {
    procesarComando(bt.read());
  }
}

void setup() {
  Serial.begin(115200);
  bt.begin(9600);
  Wire.begin();

  if (!mpu.begin(0x68)) {
    Serial.println(F("# ERROR: MPU6050 no encontrado"));
    while (true) {
      delay(100);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println(F("# AGITAF - PARCHE HC-05"));
  Serial.println(F("# Sensor MPU6050 detectado"));
  Serial.println(F("# P=Pausar, R=Reanudar"));
  Serial.println(F("# C=Nueva sesion, S=Resumen"));
  Serial.println(F("# Umbral experimental: 2.2 g"));
  Serial.println(F("tipo,tiempo_ms,aceleracion_g,giro_dps,eventos,pico_g,pico_giro_dps"));
}

void loop() {
  controlarComandos();

  if (pausado) {
    return;
  }

  unsigned long ahora = millis();

  if (ahora - ultimaLectura < INTERVALO_MS) {
    return;
  }

  ultimaLectura = ahora;

  sensors_event_t a, g, temp;

  if (!mpu.getEvent(&a, &g, &temp)) {
    mensaje(F("# ERROR DE LECTURA"));
    return;
  }

  // ACELERACION RESULTANTE EN g
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float aceleracion =
    sqrt(ax*ax + ay*ay + az*az) / 9.80665;

  // VELOCIDAD ANGULAR EN GRADOS/SEGUNDO
  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  float giro =
    sqrt(gx*gx + gy*gy + gz*gz) * 180.0 / PI;

  // DETECCION DE EVENTOS EXPERIMENTALES
  if (!registrando &&
      aceleracion >= UMBRAL_G &&
      (primerEvento ||
       ahora - ultimoEvento >= ESPERA_EVENTO_MS)) {

    registrando = true;
    inicioEvento = ahora;
    picoEvento = aceleracion;
    picoGiroEvento = giro;
  }

  if (registrando) {

    if (aceleracion > picoEvento) {
      picoEvento = aceleracion;
    }

    if (giro > picoGiroEvento) {
      picoGiroEvento = giro;
    }

    if (ahora - inicioEvento >= VENTANA_EVENTO_MS) {
      registrando = false;
      primerEvento = false;
      ultimoEvento = ahora;
      contador++;

      sumaPicos += picoEvento;

      if (picoEvento > picoMaximoSesion) {
        picoMaximoSesion = picoEvento;
      }

      // USB (formato original)
      Serial.print(F("EVENTO,"));
      Serial.print(ahora);
      Serial.print(F(",,,"));
      Serial.print(contador);
      Serial.print(',');
      Serial.print(picoEvento, 2);
      Serial.print(',');
      Serial.println(picoGiroEvento, 1);

      // Bluetooth (formato compacto): E,tiempo_ms,eventos,pico_g,pico_giro_dps
      bt.print(F("E,"));
      bt.print(ahora);
      bt.print(',');
      bt.print(contador);
      bt.print(',');
      bt.print(picoEvento, 2);
      bt.print(',');
      bt.println(picoGiroEvento, 0);
    }
  }

  // REGISTRO POR USB CADA 50 ms (formato original)
  if (ahora - ultimoReporte >= 50) {
    ultimoReporte = ahora;

    Serial.print(F("DATO,"));
    Serial.print(ahora);
    Serial.print(',');
    Serial.print(aceleracion, 3);
    Serial.print(',');
    Serial.print(giro, 2);
    Serial.print(',');
    Serial.println(contador);
  }

  // DATOS POR BLUETOOTH CADA 100 ms (formato compacto): D,tiempo_ms,aceleracion_g,giro_dps,eventos
  if (ahora - ultimoBT >= INTERVALO_BT_MS) {
    ultimoBT = ahora;

    bt.print(F("D,"));
    bt.print(ahora);
    bt.print(',');
    bt.print(aceleracion, 2);
    bt.print(',');
    bt.print(giro, 0);
    bt.print(',');
    bt.println(contador);
  }
}
