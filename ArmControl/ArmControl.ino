#include <Arduino.h>

#define PPM_PIN            2
#define RX_CHANNEL_COUNT   6
#define SYNC_PULSE_US      2100
#define BAUD_RATE          115200

// Deadzone 
#define DZ_LOW   400
#define DZ_HIGH  600

// (rad per ciclo)
#define B_STEP   0.035f
#define S_STEP   0.02f
#define E_STEP   0.02f

// Ranges
#define B_MIN   -3.14f
#define B_MAX    3.14f
#define S_MIN   -1.56f
#define S_MAX    1.56f
#define E_MIN   -0.75f
#define E_MAX    3.00f

// Gripper t range
const float T_MIN = 1.08f;
const float T_MAX = 3.23f;

// Valores iniciales (primer init)
float b_val =  0.00f;
float s_val =  0.0f;
float e_val =  1.48f;
float t_val =  3.10f;

int ppmData[RX_CHANNEL_COUNT+1];

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
  memset(ppmData, 0, sizeof(ppmData));
}

void loop() {
  // Raw 0-1000 de cada canal
  int rawZ = constrain(ppmData[2],1000,2000) - 1000;
  int rawY = constrain(ppmData[3],1000,2000) - 1000;
  int rawX = constrain(ppmData[4],1000,2000) - 1000;
  int sw   = constrain(ppmData[5],1000,2000);        // switch de habilitación
  int rawT = constrain(ppmData[6],1000,2000) - 1000; // control gripper

  // Si no está habilitado, nada cambia
  if (sw <= 1200) {
    delay(50);
    return;
  }

  // b,s,e
  if      (rawX > DZ_HIGH) b_val += B_STEP;
  else if (rawX < DZ_LOW ) b_val -= B_STEP;
  b_val = constrain(b_val, B_MIN, B_MAX);

  if      (rawY > DZ_HIGH) s_val += S_STEP;
  else if (rawY < DZ_LOW ) s_val -= S_STEP;
  s_val = constrain(s_val, S_MIN, S_MAX);

  if      (rawZ > DZ_HIGH) e_val += E_STEP;
  else if (rawZ < DZ_LOW ) e_val -= E_STEP;
  e_val = constrain(e_val, E_MIN, E_MAX);

  // Recalcula t_val
  float ratio = rawT / 1000.0f;
  t_val = T_MIN + ratio * (T_MAX - T_MIN);

  // Emite CSV: b,s,e,t
  Serial.print(b_val, 6);  Serial.print(',');
  Serial.print(s_val, 6);  Serial.print(',');
  Serial.print(e_val, 6);  Serial.print(',');
  Serial.println(t_val, 6);

  delay(50);
}

void ppmISR() {
  static unsigned long last = 0;
  static uint8_t chan = 0;
  unsigned long now = micros();
  unsigned long dt  = now - last;
  last = now;
  if (dt > SYNC_PULSE_US || chan > RX_CHANNEL_COUNT) chan = 0;
  ppmData[chan++] = dt;
}
