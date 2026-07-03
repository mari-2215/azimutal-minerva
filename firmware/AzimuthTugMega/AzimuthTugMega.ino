/*
  Azimuth Tug Mega
  Author: mari-2215

  Controla um propulsor azimutal RC com Arduino Mega, receptor PWM e servo 270 graus.

  Modelo de controle:
  - Stick de leme: ate +/- 45 graus.
  - Stick de aceleracao para frente: pod aponta para vante.
  - Stick de aceleracao para re: pod gira 180 graus e o motor recebe magnitude positiva.
*/

#include <Arduino.h>
#include <Servo.h>

// ---------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------

const byte RUDDER_INPUT_PIN = 2;    // Canal PWM do receptor: leme
const byte THROTTLE_INPUT_PIN = 3;  // Canal PWM do receptor: acelerador
const byte AZIMUTH_SERVO_PIN = 9;
const byte ESC_OUTPUT_PIN = 10;

// Faixa tipica de receptor RC. Ajuste olhando no Serial Monitor.
const int RC_MIN_US = 1000;
const int RC_CENTER_US = 1500;
const int RC_MAX_US = 2000;
const int RC_DEADBAND_US = 35;

// Falha se algum canal ficar sem pulso novo por este tempo.
const unsigned long SIGNAL_TIMEOUT_US = 120000UL;

// Servo 270 graus. Muitos modelos aceitam 500-2500 us, mas calibre o seu.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const float SERVO_TOTAL_DEGREES = 270.0f;

// Geometria do pod dentro do curso de 270 graus.
// Frente usa 0..90 graus; re usa 180..270 graus.
// Alinhe mecanicamente o pod para que 45 graus seja "frente".
const float POD_LEFT_LIMIT_DEG = 0.0f;
const float POD_FORWARD_CENTER_DEG = 45.0f;
const float POD_REVERSE_CENTER_DEG = 225.0f;
const float POD_RIGHT_LIMIT_DEG = 270.0f;
const float RUDDER_MAX_DEFLECTION_DEG = 45.0f;

// Inverta caso sua mecanica esteja espelhada.
const bool INVERT_RUDDER_INPUT = false;
const bool INVERT_THROTTLE_INPUT = false;
const bool INVERT_SERVO_OUTPUT = false;

// ESC/motor. O firmware manda motor sempre para frente, usando modulo do stick.
const int ESC_STOP_US = 1000;
const int ESC_MAX_US = 2000;
const int ESC_ARM_US = 1000;
const unsigned long ESC_ARM_TIME_MS = 2500UL;

// Suavizacao por ciclo de loop.
const float SERVO_MAX_STEP_DEG = 1.8f;
const int ESC_MAX_STEP_US = 6;

// Debug por serial.
const bool SERIAL_DEBUG = true;
const unsigned long DEBUG_INTERVAL_MS = 250UL;

// ---------------------------------------------------------------------------
// PWM input capture
// ---------------------------------------------------------------------------

struct RcChannel {
  byte pin;
  volatile unsigned long riseUs;
  volatile unsigned int pulseUs;
  volatile unsigned long lastPulseUs;
};

RcChannel rudderChannel = {RUDDER_INPUT_PIN, 0, RC_CENTER_US, 0};
RcChannel throttleChannel = {THROTTLE_INPUT_PIN, 0, RC_CENTER_US, 0};

Servo azimuthServo;
Servo esc;

float currentServoDeg = POD_FORWARD_CENTER_DEG;
int currentEscUs = ESC_STOP_US;
unsigned long lastDebugMs = 0;

void captureChannel(RcChannel &channel) {
  const unsigned long now = micros();

  if (digitalRead(channel.pin) == HIGH) {
    channel.riseUs = now;
    return;
  }

  const unsigned long width = now - channel.riseUs;
  if (width >= 800UL && width <= 2200UL) {
    channel.pulseUs = (unsigned int)width;
    channel.lastPulseUs = now;
  }
}

void onRudderChange() {
  captureChannel(rudderChannel);
}

void onThrottleChange() {
  captureChannel(throttleChannel);
}

unsigned int readPulseAtomic(const RcChannel &channel) {
  noInterrupts();
  const unsigned int pulse = channel.pulseUs;
  interrupts();
  return pulse;
}

unsigned long readLastPulseAtomic(const RcChannel &channel) {
  noInterrupts();
  const unsigned long lastPulse = channel.lastPulseUs;
  interrupts();
  return lastPulse;
}

// ---------------------------------------------------------------------------
// Control helpers
// ---------------------------------------------------------------------------

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float normalizeRc(unsigned int pulseUs, bool invert) {
  int centered = (int)pulseUs - RC_CENTER_US;

  if (abs(centered) <= RC_DEADBAND_US) {
    return 0.0f;
  }

  const int span = centered > 0 ? (RC_MAX_US - RC_CENTER_US) : (RC_CENTER_US - RC_MIN_US);
  float normalized = (float)centered / (float)span;
  normalized = clampFloat(normalized, -1.0f, 1.0f);

  return invert ? -normalized : normalized;
}

bool signalHealthy() {
  const unsigned long now = micros();
  const unsigned long rudderAge = now - readLastPulseAtomic(rudderChannel);
  const unsigned long throttleAge = now - readLastPulseAtomic(throttleChannel);
  return rudderAge < SIGNAL_TIMEOUT_US && throttleAge < SIGNAL_TIMEOUT_US;
}

float servoDegreesToMicros(float degrees) {
  degrees = clampFloat(degrees, 0.0f, SERVO_TOTAL_DEGREES);

  if (INVERT_SERVO_OUTPUT) {
    degrees = SERVO_TOTAL_DEGREES - degrees;
  }

  return SERVO_MIN_US + (degrees / SERVO_TOTAL_DEGREES) * (SERVO_MAX_US - SERVO_MIN_US);
}

float approachFloat(float current, float target, float maxStep) {
  const float delta = target - current;
  if (abs(delta) <= maxStep) {
    return target;
  }
  return current + (delta > 0.0f ? maxStep : -maxStep);
}

int approachInt(int current, int target, int maxStep) {
  const int delta = target - current;
  if (abs(delta) <= maxStep) {
    return target;
  }
  return current + (delta > 0 ? maxStep : -maxStep);
}

float computeTargetPodDeg(float rudder, float throttle) {
  const float deflection = rudder * RUDDER_MAX_DEFLECTION_DEG;

  if (throttle < 0.0f) {
    return clampFloat(POD_REVERSE_CENTER_DEG - deflection, POD_LEFT_LIMIT_DEG, POD_RIGHT_LIMIT_DEG);
  }

  return clampFloat(POD_FORWARD_CENTER_DEG + deflection, POD_LEFT_LIMIT_DEG, POD_RIGHT_LIMIT_DEG);
}

int computeTargetEscUs(float throttle) {
  const float magnitude = abs(throttle);

  if (magnitude <= 0.02f) {
    return ESC_STOP_US;
  }

  return ESC_STOP_US + (int)(magnitude * (ESC_MAX_US - ESC_STOP_US));
}

void writeOutputs(float targetServoDeg, int targetEscUs) {
  currentServoDeg = approachFloat(currentServoDeg, targetServoDeg, SERVO_MAX_STEP_DEG);
  currentEscUs = approachInt(currentEscUs, targetEscUs, ESC_MAX_STEP_US);

  azimuthServo.writeMicroseconds((int)servoDegreesToMicros(currentServoDeg));
  esc.writeMicroseconds(currentEscUs);
}

void printDebug(unsigned int rudderUs, unsigned int throttleUs, float rudder, float throttle, float targetDeg, int targetEscUs, bool healthy) {
  if (!SERIAL_DEBUG) return;

  const unsigned long now = millis();
  if (now - lastDebugMs < DEBUG_INTERVAL_MS) return;
  lastDebugMs = now;

  Serial.print(F("healthy="));
  Serial.print(healthy ? F("yes") : F("no"));
  Serial.print(F(" rudderUs="));
  Serial.print(rudderUs);
  Serial.print(F(" throttleUs="));
  Serial.print(throttleUs);
  Serial.print(F(" rudder="));
  Serial.print(rudder, 2);
  Serial.print(F(" throttle="));
  Serial.print(throttle, 2);
  Serial.print(F(" targetDeg="));
  Serial.print(targetDeg, 1);
  Serial.print(F(" servoDeg="));
  Serial.print(currentServoDeg, 1);
  Serial.print(F(" escUs="));
  Serial.print(currentEscUs);
  Serial.print(F(" targetEscUs="));
  Serial.println(targetEscUs);
}

// ---------------------------------------------------------------------------
// Arduino lifecycle
// ---------------------------------------------------------------------------

void setup() {
  pinMode(RUDDER_INPUT_PIN, INPUT);
  pinMode(THROTTLE_INPUT_PIN, INPUT);

  if (SERIAL_DEBUG) {
    Serial.begin(115200);
  }

  azimuthServo.attach(AZIMUTH_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  esc.attach(ESC_OUTPUT_PIN);

  azimuthServo.writeMicroseconds((int)servoDegreesToMicros(POD_FORWARD_CENTER_DEG));
  esc.writeMicroseconds(ESC_ARM_US);
  delay(ESC_ARM_TIME_MS);

  attachInterrupt(digitalPinToInterrupt(RUDDER_INPUT_PIN), onRudderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_INPUT_PIN), onThrottleChange, CHANGE);
}

void loop() {
  const unsigned int rudderUs = readPulseAtomic(rudderChannel);
  const unsigned int throttleUs = readPulseAtomic(throttleChannel);
  const bool healthy = signalHealthy();

  float rudder = 0.0f;
  float throttle = 0.0f;
  float targetPodDeg = POD_FORWARD_CENTER_DEG;
  int targetEscUs = ESC_STOP_US;

  if (healthy) {
    rudder = normalizeRc(rudderUs, INVERT_RUDDER_INPUT);
    throttle = normalizeRc(throttleUs, INVERT_THROTTLE_INPUT);
    targetPodDeg = computeTargetPodDeg(rudder, throttle);
    targetEscUs = computeTargetEscUs(throttle);
  }

  writeOutputs(targetPodDeg, targetEscUs);
  printDebug(rudderUs, throttleUs, rudder, throttle, targetPodDeg, targetEscUs, healthy);

  delay(10);
}
