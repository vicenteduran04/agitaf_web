
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

// INNOVA - Dos sensores a 50 Hz
// Sensor 1: 0x68
// Sensor 2: 0x69

const byte DIR[2] = {0x68, 0x69};
const uint32_t FIRMA = 0x494E4E32UL;
const uint32_t INTERVALO_US = 20000UL; // 50 Hz

struct Calibracion {
  uint32_t firma;
  float offset[2][3];
  float escala[2][3];
};

struct Medicion {
  float ax, ay, az;
  float gx, gy, gz;
};

Calibracion cal;
float offsetG[2][3] = {};

uint32_t inicioUS;
uint32_t proximaMuestra;

bool escribir(byte dir, byte reg, byte valor) {
  Wire.beginTransmission(dir);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

bool iniciar(byte dir) {
  if (!escribir(dir, 0x6B, 0x01)) return false;
  delay(100);

  // Filtro digital
  if (!escribir(dir, 0x1A, 0x03)) return false;

  // Frecuencia interna 50 Hz
  if (!escribir(dir, 0x19, 19)) return false;

  // Giroscopio +/- 500 grados/s
  if (!escribir(dir, 0x1B, 0x08)) return false;

  // Acelerometro +/- 8 g
  if (!escribir(dir, 0x1C, 0x10)) return false;

  return true;
}

int16_t leer16() {
  uint16_t alto = (uint8_t)Wire.read();
  uint16_t bajo = (uint8_t)Wire.read();
  return (int16_t)((alto << 8) | bajo);
}

bool leerSensor(byte s, Medicion &m) {
  Wire.beginTransmission(DIR[s]);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(DIR[s], (byte)14) != 14) {
    return false;
  }

  int16_t ax = leer16();
  int16_t ay = leer16();
  int16_t az = leer16();

  leer16(); // Temperatura

  int16_t gx = leer16();
  int16_t gy = leer16();
  int16_t gz = leer16();

  // Aceleracion calibrada en g
  m.ax = (ax / 4096.0f - cal.offset[s][0])
         * cal.escala[s][0];

  m.ay = (ay / 4096.0f - cal.offset[s][1])
         * cal.escala[s][1];

  m.az = (az / 4096.0f - cal.offset[s][2])
         * cal.escala[s][2];

  // Giroscopio en grados por segundo
  m.gx = gx / 65.5f;
  m.gy = gy / 65.5f;
  m.gz = gz / 65.5f;

  return true;
}

bool calibrarGiroscopios() {
  float sumas[2][3] = {};
  const int N = 300;

  Serial.println("Calibrando giroscopios...");
  Serial.println("NO MOVER LOS SENSORES");

  for (int n = 0; n < N; n++) {
    Medicion m1, m2;

    if (!leerSensor(0, m1) ||
        !leerSensor(1, m2)) {
      return false;
    }

    sumas[0][0] += m1.gx;
    sumas[0][1] += m1.gy;
    sumas[0][2] += m1.gz;

    sumas[1][0] += m2.gx;
    sumas[1][1] += m2.gy;
    sumas[1][2] += m2.gz;

    delay(10);
  }

  for (int s = 0; s < 2; s++) {
    for (int e = 0; e < 3; e++) {
      offsetG[s][e] = sumas[s][e] / N;
    }
  }

  return true;
}

float magnitud(const Medicion &m) {
  return sqrt(m.ax*m.ax + m.ay*m.ay + m.az*m.az);
}

void imprimirSensor(const Medicion &m, byte s) {
  Serial.print(',');
  Serial.print(m.ax, 3);
  Serial.print(',');
  Serial.print(m.ay, 3);
  Serial.print(',');
  Serial.print(m.az, 3);
  Serial.print(',');
  Serial.print(magnitud(m), 3);

  Serial.print(',');
  Serial.print(m.gx - offsetG[s][0], 2);
  Serial.print(',');
  Serial.print(m.gy - offsetG[s][1], 2);
  Serial.print(',');
  Serial.print(m.gz - offsetG[s][2], 2);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  delay(1000);

  EEPROM.get(0, cal);

  if (cal.firma != FIRMA) {
    Serial.println("ERROR: EEPROM no calibrada");
    while (true) delay(1000);
  }

  for (int s = 0; s < 2; s++) {
    for (int e = 0; e < 3; e++) {
      if (!(cal.escala[s][e] > 0.5f &&
            cal.escala[s][e] < 1.5f)) {
        Serial.println("ERROR: calibracion invalida");
        while (true) delay(1000);
      }
    }
  }

  if (!iniciar(DIR[0]) || !iniciar(DIR[1])) {
    Serial.println("ERROR al iniciar sensores");
    while (true) delay(1000);
  }

  if (!calibrarGiroscopios()) {
    Serial.println("ERROR en calibracion");
    while (true) delay(1000);
  }

  Serial.println("Calibracion completada");
  Serial.println("INNOVA - 50 Hz");

  // Una fila contiene las lecturas de ambos sensores
  Serial.println(
    "t_ms,ax1,ay1,az1,a1_g,gx1,gy1,gz1,"
    "ax2,ay2,az2,a2_g,gx2,gy2,gz2"
  );

  inicioUS = micros();
  proximaMuestra = inicioUS;
}

void loop() {
  uint32_t ahora = micros();

  if ((int32_t)(ahora - proximaMuestra) < 0) {
    return;
  }

  uint32_t t_ms = (ahora - inicioUS) / 1000UL;
  proximaMuestra += INTERVALO_US;

  Medicion m1, m2;

  if (leerSensor(0, m1) && leerSensor(1, m2)) {
    Serial.print(t_ms);
    imprimirSensor(m1, 0);
    imprimirSensor(m2, 1);
    Serial.println();
  } else {
    Serial.println("# ERROR DE LECTURA I2C");
  }

  // Evitar acumular retrasos si una lectura tarda demasiado
  if ((int32_t)(micros() - proximaMuestra) >= 0) {
    proximaMuestra = micros() + INTERVALO_US;
  }
}
