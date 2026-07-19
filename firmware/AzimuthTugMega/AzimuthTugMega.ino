/*
  Minerva Integrado - Arduino Mega 2560
  Controle azimutal + telemetria em um unico firmware.

  MAPEAMENTO DO CONTROLE:
    CH4 receptor -> D2: joystick esquerdo horizontal, direcao +/-45 graus
    CH3 receptor -> D3: joystick esquerdo vertical, latch frente/re (180 graus)
    CH2 receptor -> D18: joystick direito vertical, potencia do propeller

  SAIDAS:
    D9  -> servo azimutal MG946R 270 graus
    D10 -> sinal do ESC

  USB Serial:
    Arduino Mega <-> Raspberry Pi, 115200 baud
    Formato binario MinervaFrame v1 com CRC-16/CCITT.

  PINAGEM DOS SENSORES:
    A0  <- LM35
    A3  <- ACS712
    A4  <- divisor de tensao 0-25 V
    D22 <- DHT11
    D23 <- sensor de agua (ativo em LOW)

    Serial2:
      RX2 D17 <- TX do GPS NEO-6M
      TX2 D16 -> RX do GPS (opcional)

    D18 fica reservado para o sinal do CH2.

    I2C:
      SDA D20 / SCL D21 -> ADXL345

  IMPORTANTE:
    - Servo e ESC devem usar alimentacao/BEC apropriado.
    - Todos os GNDs devem estar em comum.
    - Nao alimente o servo forte pelo pino 5 V do Arduino.
    - Teste inicialmente com helice removida ou motor desconectado.
*/

#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// CONFIGURACAO DO CONTROLE AZIMUTAL
// ============================================================================

// CH4: horizontal esquerdo -> +/-45 graus.
constexpr uint8_t RUDDER_INPUT_PIN = 2;

// CH3: vertical esquerdo -> seleciona e trava frente/re.
constexpr uint8_t DIRECTION_INPUT_PIN = 3;

// CH2: vertical direito -> potencia do propeller.
// D18 possui interrupcao externa no Arduino Mega.
constexpr uint8_t PROPULSION_INPUT_PIN = 18;

constexpr uint8_t AZIMUTH_SERVO_PIN = 9;
constexpr uint8_t ESC_OUTPUT_PIN = 10;

constexpr int RC_MIN_US = 500;
constexpr int RC_CENTER_US = 1500;
constexpr int RC_MAX_US = 2500;
constexpr int RC_DEADBAND_US = 35;

constexpr uint32_t SIGNAL_TIMEOUT_US = 250000UL;

// Calibre estes dois valores sem forcar o fim mecanico do servo.
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr float SERVO_TOTAL_DEGREES = 270.0F;

// Geometria do pod:
//   frente: 0 a 90 graus, centro em 45
//   re:     180 a 270 graus, centro em 225
constexpr float POD_LEFT_LIMIT_DEG = 0.0F;
constexpr float POD_FORWARD_CENTER_DEG = 45.0F;
constexpr float POD_REVERSE_CENTER_DEG = 225.0F;
constexpr float POD_RIGHT_LIMIT_DEG = 270.0F;
constexpr float RUDDER_MAX_DEFLECTION_DEG = 45.0F;

// LATCH DA DIRECAO:
// O vertical nao controla o angulo continuamente. Ele apenas seleciona
// e trava FRENTE ou RE quando passa do limiar durante alguns milissegundos.
constexpr float DIRECTION_SELECT_THRESHOLD = 0.35F;
constexpr uint32_t DIRECTION_CONFIRM_MS = 180UL;

// A troca de 180 graus altera somente o alvo do servo.
    // O ESC nunca consulta este estado.

// Filtro somente no horizontal para reduzir tremedeira sem deixar pesado.
constexpr float RUDDER_FILTER_ALPHA = 0.25F;
constexpr float SERVO_COMMAND_EPSILON_DEG = 0.35F;

constexpr bool INVERT_RUDDER_INPUT = false;
constexpr bool INVERT_DIRECTION_INPUT = false;

// O vertical direito e autocentrante:
// centro = motor parado; empurrar para cima = aumenta potencia.
// Se estiver acelerando para baixo, troque para true.
constexpr bool INVERT_PROPULSION_INPUT = false;

constexpr bool INVERT_SERVO_OUTPUT = false;

// ESC brushed bidirecional medido no sistema:
//   1500 us = neutro
//   acima de 1600 us = propulsao para frente
//   2500 us = maximo
constexpr int ESC_STOP_US = 1500;
constexpr int ESC_START_US = 1600;
constexpr int ESC_MAX_US = 2500;
constexpr int ESC_MAX_SNAP_US = 2450;
constexpr int ESC_ARM_US = 1500;
constexpr uint32_t ESC_ARM_TIME_MS = 5000UL;

// Protecao LOCAL do CH2: exige neutro na inicializacao e tres
// amostras novas consecutivas antes de iniciar o motor.
constexpr uint32_t PROPULSION_ARM_NEUTRAL_MS = 500UL;
constexpr uint8_t PROPULSION_START_CONFIRM_SAMPLES = 3;

// Controle a 100 Hz.
constexpr uint32_t CONTROL_INTERVAL_MS = 10UL;

// ============================================================================
// CONFIGURACAO DA TELEMETRIA
// ============================================================================

constexpr uint8_t LM35_PIN = A0;
constexpr uint8_t CURRENT_PIN = A3;
constexpr uint8_t VOLTAGE_PIN = A4;
constexpr uint8_t DHT_PIN = 22;
constexpr uint8_t WATER_PIN = 23;

constexpr uint32_t TELEMETRY_INTERVAL_MS = 500UL;
constexpr uint32_t DHT_INTERVAL_MS = 2000UL;

constexpr float ADC_REFERENCE_V = 5.0F;
constexpr float VOLTAGE_DIVIDER_RATIO = 5.0F;

// ACS712-5A. Troque conforme seu modulo:
//   5 A  -> 0.185 V/A
//   20 A -> 0.100 V/A
//   30 A -> 0.066 V/A
constexpr float ACS_ZERO_V = 2.5F;
constexpr float ACS_SENSITIVITY_V_PER_A = 0.185F;

constexpr float CRITICAL_BATTERY_V = 10.8F;
constexpr char BOAT_ID[] = "azimutal-01";

// ============================================================================
// PROTOCOLO MINERVA FRAME V1
// ============================================================================

constexpr uint8_t FRAME_PREAMBLE_0 = 0xA5;
constexpr uint8_t FRAME_PREAMBLE_1 = 0x5A;
constexpr uint8_t FRAME_VERSION = 1;
constexpr uint8_t FRAME_TYPE_TELEMETRY = 1;
constexpr size_t FRAME_MAX_PAYLOAD_BYTES = 1024;

// ============================================================================
// OBJETOS E ESTADO GLOBAL
// ============================================================================

struct RcChannel {
  uint8_t pin;
  volatile uint32_t riseUs;
  volatile uint16_t pulseUs;
  volatile uint32_t lastPulseUs;
  volatile uint32_t sampleCounter;
};

struct RcSnapshot {
  uint16_t rudderUs;
  uint16_t directionUs;
  uint16_t propulsionUs;

  uint32_t rudderLastUs;
  uint32_t directionLastUs;
  uint32_t propulsionLastUs;

  uint32_t rudderCounter;
  uint32_t directionCounter;
  uint32_t propulsionCounter;
};

RcChannel rudderChannel = {
  RUDDER_INPUT_PIN,
  0,
  RC_CENTER_US,
  0,
  0
};

RcChannel directionChannel = {
  DIRECTION_INPUT_PIN,
  0,
  RC_CENTER_US,
  0,
  0
};

RcChannel propulsionChannel = {
  PROPULSION_INPUT_PIN,
  0,
  RC_CENTER_US,
  0,
  0
};

Servo azimuthServo;
Servo esc;

DHT dht(DHT_PIN, DHT11);
TinyGPSPlus gps;
Adafruit_ADXL345_Unified accelerometer(34501);

bool accelerometerReady = false;

// Estado travado da direcao.
// false = frente; true = re.
bool reverseMode = false;
bool pendingReverseMode = false;
bool directionCandidateActive = false;
bool directionChangeInProgress = false;
uint32_t directionCandidateSinceMs = 0;
uint32_t directionChangeStartedMs = 0;

float filteredRudderNormalized = 0.0F;
float lastServoCommandDeg = POD_FORWARD_CENTER_DEG;

float currentServoDeg = POD_FORWARD_CENTER_DEG;
int currentServoUs = 0;
int currentEscUs = ESC_STOP_US;

float rudderNormalized = 0.0F;
float directionNormalized = 0.0F;
float propulsionNormalized = 0.0F;
float targetPodAngleDeg = POD_FORWARD_CENTER_DEG;

// Estes dois campos existem somente para telemetria.
bool rcHealthy = false;
bool failsafeActive = true;

// Estado EXCLUSIVO da propulsao CH2.
// Nenhuma funcao do servo le ou altera estas variaveis.
bool propulsionArmed = false;
bool propulsionNeutralTimerActive = false;
uint32_t propulsionNeutralSinceMs = 0;

bool propulsionActive = false;
uint8_t propulsionStartConfirmCount = 0;
uint32_t lastProcessedPropulsionCounter = 0;

float cachedAirTempC = NAN;
float cachedHumidityPct = NAN;

uint32_t sequenceNumber = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastDhtMs = 0;

// ============================================================================
// FUNCOES AUXILIARES
// ============================================================================

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int approachInt(int current, int target, int maxStep) {
  const int delta = target - current;

  if (abs(delta) <= maxStep) {
    return target;
  }

  return current + (delta > 0 ? maxStep : -maxStep);
}

float readAveragedAdc(uint8_t pin, uint8_t samples = 16) {
  uint32_t sum = 0;

  for (uint8_t index = 0; index < samples; ++index) {
    sum += analogRead(pin);
  }

  return static_cast<float>(sum) /
         static_cast<float>(samples);
}

float adcToVolts(float adc) {
  return adc * ADC_REFERENCE_V / 1023.0F;
}

// ============================================================================
// CAPTURA DOS CANAIS PWM DO RECEPTOR
// ============================================================================

void captureChannel(RcChannel &channel) {
  const uint32_t nowUs = micros();

  if (digitalRead(channel.pin) == HIGH) {
    channel.riseUs = nowUs;
    return;
  }

  const uint32_t widthUs = nowUs - channel.riseUs;

  if (widthUs >= 400UL && widthUs <= 2600UL) {
    channel.pulseUs = static_cast<uint16_t>(widthUs);
    channel.lastPulseUs = nowUs;
    ++channel.sampleCounter;
  }
}

void onRudderChange() {
  captureChannel(rudderChannel);
}

void onDirectionChange() {
  captureChannel(directionChannel);
}

void onPropulsionChange() {
  captureChannel(propulsionChannel);
}

RcSnapshot readRcSnapshot() {
  RcSnapshot snapshot;

  noInterrupts();

  snapshot.rudderUs = rudderChannel.pulseUs;
  snapshot.directionUs = directionChannel.pulseUs;
  snapshot.propulsionUs = propulsionChannel.pulseUs;

  snapshot.rudderLastUs = rudderChannel.lastPulseUs;
  snapshot.directionLastUs = directionChannel.lastPulseUs;
  snapshot.propulsionLastUs = propulsionChannel.lastPulseUs;

  snapshot.rudderCounter = rudderChannel.sampleCounter;
  snapshot.directionCounter = directionChannel.sampleCounter;
  snapshot.propulsionCounter = propulsionChannel.sampleCounter;

  interrupts();

  return snapshot;
}

bool isChannelHealthy(
    uint32_t lastPulseUs,
    uint32_t nowUs) {

  if (lastPulseUs == 0) {
    return false;
  }

  return
      (nowUs - lastPulseUs) <
      SIGNAL_TIMEOUT_US;
}

float normalizeRc(uint16_t pulseUs, bool invert) {
  const int centered =
      static_cast<int>(pulseUs) - RC_CENTER_US;

  if (abs(centered) <= RC_DEADBAND_US) {
    return 0.0F;
  }

  const int span =
      centered > 0
          ? RC_MAX_US - RC_CENTER_US
          : RC_CENTER_US - RC_MIN_US;

  float normalized =
      static_cast<float>(centered) /
      static_cast<float>(span);

  normalized = clampFloat(
    normalized,
    -1.0F,
    1.0F
  );

  return invert ? -normalized : normalized;
}

// Valor normalizado apenas para telemetria.
// A saida real do ESC usa diretamente o pulso valido do CH2.
float normalizePropulsionRc(uint16_t pulseUs, bool invert) {
  int pulse = static_cast<int>(pulseUs);

  if (invert) {
    pulse =
        RC_CENTER_US -
        (pulse - RC_CENTER_US);
  }

  if (pulse < ESC_START_US) {
    return 0.0F;
  }

  if (pulse >= ESC_MAX_SNAP_US) {
    return 1.0F;
  }

  const float normalized =
      static_cast<float>(pulse - ESC_START_US) /
      static_cast<float>(ESC_MAX_US - ESC_START_US);

  return clampFloat(normalized, 0.0F, 1.0F);
}

// ============================================================================
// CONTROLE DO SERVO E DO ESC
// ============================================================================

int servoDegreesToMicros(float degrees) {
  degrees = clampFloat(
    degrees,
    0.0F,
    SERVO_TOTAL_DEGREES
  );

  if (INVERT_SERVO_OUTPUT) {
    degrees = SERVO_TOTAL_DEGREES - degrees;
  }

  const float pulse =
      SERVO_MIN_US +
      (degrees / SERVO_TOTAL_DEGREES) *
      (SERVO_MAX_US - SERVO_MIN_US);

  return static_cast<int>(pulse + 0.5F);
}

void updateDirectionLatch(float vertical, uint32_t nowMs) {
  bool hasCandidate = false;
  bool candidateReverse = reverseMode;

  // Vertical para baixo solicita RE.
  if (vertical <= -DIRECTION_SELECT_THRESHOLD) {
    candidateReverse = true;
    hasCandidate = true;
  }

  // Vertical para cima solicita FRENTE.
  else if (vertical >= DIRECTION_SELECT_THRESHOLD) {
    candidateReverse = false;
    hasCandidate = true;
  }

  // Perto do centro, mantem o ultimo estado travado.
  if (!hasCandidate) {
    directionCandidateActive = false;
    return;
  }

  // Ja esta no sentido solicitado.
  if (candidateReverse == reverseMode) {
    directionCandidateActive = false;
    return;
  }

  // Comecou uma nova solicitacao de troca.
  if (!directionCandidateActive ||
      pendingReverseMode != candidateReverse) {

    pendingReverseMode = candidateReverse;
    directionCandidateSinceMs = nowMs;
    directionCandidateActive = true;
    return;
  }

  // Confirma a troca apenas se o joystick permanecer firme.
  if (nowMs - directionCandidateSinceMs >=
      DIRECTION_CONFIRM_MS) {

    reverseMode = pendingReverseMode;
    directionCandidateActive = false;

    // Somente o servo usa reverseMode.
    // O ESC nao e parado, rearmado ou modificado aqui.
  }
}

float computeTargetPodDeg(float horizontal) {
  const float deflection =
      horizontal * RUDDER_MAX_DEFLECTION_DEG;

  if (reverseMode) {
    return clampFloat(
      POD_REVERSE_CENTER_DEG - deflection,
      POD_LEFT_LIMIT_DEG,
      POD_RIGHT_LIMIT_DEG
    );
  }

  return clampFloat(
    POD_FORWARD_CENTER_DEG + deflection,
    POD_LEFT_LIMIT_DEG,
    POD_RIGHT_LIMIT_DEG
  );
}

int computeTargetEscUs(uint16_t propulsionUs) {
  int pulse = static_cast<int>(propulsionUs);

  if (INVERT_PROPULSION_INPUT) {
    pulse =
        RC_CENTER_US -
        (pulse - RC_CENTER_US);
  }

  if (pulse < ESC_START_US) {
    return ESC_STOP_US;
  }

  if (pulse >= ESC_MAX_SNAP_US) {
    return ESC_MAX_US;
  }

  return constrain(
    pulse,
    ESC_START_US,
    ESC_MAX_US
  );
}

void resetPropulsionControl() {
  propulsionArmed = false;
  propulsionNeutralTimerActive = false;

  propulsionActive = false;
  propulsionStartConfirmCount = 0;

  currentEscUs = ESC_STOP_US;
  esc.writeMicroseconds(currentEscUs);
}

void updatePropulsionControl(
    bool propulsionHealthy,
    uint16_t propulsionUs,
    uint32_t propulsionCounter,
    uint32_t nowMs) {

  // Esta funcao recebe SOMENTE dados do CH2.
  // Nao existe acesso a CH3, CH4, reverseMode ou estado do servo.

  if (!propulsionHealthy) {
    resetPropulsionControl();
    propulsionNormalized = 0.0F;
    return;
  }

  propulsionNormalized =
      normalizePropulsionRc(
        propulsionUs,
        INVERT_PROPULSION_INPUT
      );

  int effectivePulse =
      static_cast<int>(propulsionUs);

  if (INVERT_PROPULSION_INPUT) {
    effectivePulse =
        RC_CENTER_US -
        (effectivePulse - RC_CENTER_US);
  }

  // Rearme local: depois de ligar ou perder CH2,
  // exige somente que o proprio CH2 fique neutro.
  if (!propulsionArmed) {
    if (effectivePulse <= RC_CENTER_US + RC_DEADBAND_US) {
      if (!propulsionNeutralTimerActive) {
        propulsionNeutralTimerActive = true;
        propulsionNeutralSinceMs = nowMs;
      }

      if (nowMs - propulsionNeutralSinceMs >=
          PROPULSION_ARM_NEUTRAL_MS) {

        propulsionArmed = true;
        propulsionNeutralTimerActive = false;
      }
    } else {
      propulsionNeutralTimerActive = false;
    }

    propulsionActive = false;
    propulsionStartConfirmCount = 0;
    currentEscUs = ESC_STOP_US;
    esc.writeMicroseconds(currentEscUs);
    return;
  }

  // Neutro e metade inferior param imediatamente.
  if (effectivePulse < ESC_START_US) {
    propulsionActive = false;
    propulsionStartConfirmCount = 0;
    currentEscUs = ESC_STOP_US;
    esc.writeMicroseconds(currentEscUs);
    return;
  }

  // Para evitar partida por um pulso isolado,
  // conta apenas amostras NOVAS do CH2.
  if (!propulsionActive &&
      propulsionCounter !=
          lastProcessedPropulsionCounter) {

    lastProcessedPropulsionCounter =
        propulsionCounter;

    if (propulsionStartConfirmCount < 255) {
      ++propulsionStartConfirmCount;
    }

    if (propulsionStartConfirmCount >=
        PROPULSION_START_CONFIRM_SAMPLES) {

      propulsionActive = true;
    }
  }

  if (!propulsionActive) {
    currentEscUs = ESC_STOP_US;
    esc.writeMicroseconds(currentEscUs);
    return;
  }

  currentEscUs =
      computeTargetEscUs(propulsionUs);

  esc.writeMicroseconds(currentEscUs);
}

void updateAzimuthControl() {
  const RcSnapshot snapshot = readRcSnapshot();

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // Cada canal possui saude propria.
  const bool rudderHealthy =
      isChannelHealthy(
        snapshot.rudderLastUs,
        nowUs
      );

  const bool directionHealthy =
      isChannelHealthy(
        snapshot.directionLastUs,
        nowUs
      );

  const bool propulsionHealthy =
      isChannelHealthy(
        snapshot.propulsionLastUs,
        nowUs
      );

  // Apenas para telemetria. Nao controla nenhum atuador.
  rcHealthy =
      rudderHealthy &&
      directionHealthy &&
      propulsionHealthy;

  failsafeActive = !rcHealthy;

  // ------------------------------------------------------------------------
  // CH4: SOMENTE LEME
  // ------------------------------------------------------------------------
  if (rudderHealthy) {
    rudderNormalized =
        normalizeRc(
          snapshot.rudderUs,
          INVERT_RUDDER_INPUT
        );

    filteredRudderNormalized +=
        RUDDER_FILTER_ALPHA *
        (
          rudderNormalized -
          filteredRudderNormalized
        );
  }

  // Se CH4 falhar, mantem o ultimo valor filtrado.
  // Nao mexe no motor e nao retorna o servo sozinho.

  // ------------------------------------------------------------------------
  // CH3: SOMENTE ORIENTACAO FRENTE / RE
  // ------------------------------------------------------------------------
  if (directionHealthy) {
    directionNormalized =
        normalizeRc(
          snapshot.directionUs,
          INVERT_DIRECTION_INPUT
        );

    updateDirectionLatch(
      directionNormalized,
      nowMs
    );
  } else {
    directionCandidateActive = false;
  }

  // CH3 e CH4 se encontram somente aqui, porque ambos comandam
  // o MESMO servo fisico. O motor nao participa desta conta.
  if (rudderHealthy || directionHealthy) {
    targetPodAngleDeg =
        computeTargetPodDeg(
          filteredRudderNormalized
        );

    if (fabs(
          targetPodAngleDeg -
          lastServoCommandDeg
        ) >= SERVO_COMMAND_EPSILON_DEG) {

      lastServoCommandDeg =
          targetPodAngleDeg;

      currentServoUs =
          servoDegreesToMicros(
            lastServoCommandDeg
          );

      azimuthServo.writeMicroseconds(
        currentServoUs
      );
    }
  } else {
    // Servo posicional segura a ultima posicao valida.
    azimuthServo.writeMicroseconds(
      currentServoUs
    );
  }

  currentServoDeg =
      lastServoCommandDeg;

  // ------------------------------------------------------------------------
  // CH2: SOMENTE PROPULSAO
  // ------------------------------------------------------------------------
  updatePropulsionControl(
    propulsionHealthy,
    snapshot.propulsionUs,
    snapshot.propulsionCounter,
    nowMs
  );
}

// ============================================================================
// GPS, DHT E ACELEROMETRO
// ============================================================================

void updateGps() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

void updateDht(uint32_t nowMs) {
  if (nowMs - lastDhtMs < DHT_INTERVAL_MS) {
    return;
  }

  lastDhtMs = nowMs;

  const float humidity = dht.readHumidity();
  const float temperature = dht.readTemperature();

  if (!isnan(humidity)) {
    cachedHumidityPct = humidity;
  }

  if (!isnan(temperature)) {
    cachedAirTempC = temperature;
  }
}

// ============================================================================
// CRC-16/CCITT E ESCRITA DO QUADRO BINARIO
// ============================================================================

uint16_t crc16CcittUpdate(
    uint16_t crc,
    uint8_t value) {

  crc ^= static_cast<uint16_t>(value) << 8;

  for (uint8_t bit = 0; bit < 8; ++bit) {
    if (crc & 0x8000) {
      crc = static_cast<uint16_t>(
        (crc << 1) ^ 0x1021
      );
    } else {
      crc = static_cast<uint16_t>(crc << 1);
    }
  }

  return crc;
}

void putUint16Le(
    uint8_t *destination,
    uint16_t value) {

  destination[0] =
      static_cast<uint8_t>(value & 0xFF);

  destination[1] =
      static_cast<uint8_t>((value >> 8) & 0xFF);
}

void putUint32Le(
    uint8_t *destination,
    uint32_t value) {

  destination[0] =
      static_cast<uint8_t>(value & 0xFF);

  destination[1] =
      static_cast<uint8_t>((value >> 8) & 0xFF);

  destination[2] =
      static_cast<uint8_t>((value >> 16) & 0xFF);

  destination[3] =
      static_cast<uint8_t>((value >> 24) & 0xFF);
}

void writeMinervaTelemetryFrame(
    const uint8_t *payload,
    uint16_t payloadLength,
    uint32_t sequence,
    uint32_t monotonicMs) {

  uint8_t header[12];

  header[0] = FRAME_VERSION;
  header[1] = FRAME_TYPE_TELEMETRY;

  putUint16Le(&header[2], payloadLength);
  putUint32Le(&header[4], sequence);
  putUint32Le(&header[8], monotonicMs);

  uint16_t crc = 0xFFFF;

  for (size_t index = 0;
       index < sizeof(header);
       ++index) {

    crc = crc16CcittUpdate(
      crc,
      header[index]
    );
  }

  for (uint16_t index = 0;
       index < payloadLength;
       ++index) {

    crc = crc16CcittUpdate(
      crc,
      payload[index]
    );
  }

  Serial.write(FRAME_PREAMBLE_0);
  Serial.write(FRAME_PREAMBLE_1);
  Serial.write(header, sizeof(header));
  Serial.write(payload, payloadLength);

  uint8_t crcBytes[2];
  putUint16Le(crcBytes, crc);

  Serial.write(crcBytes, sizeof(crcBytes));
}

// ============================================================================
// CONSTRUCAO E TRANSMISSAO DA TELEMETRIA
// ============================================================================

void addGpsToDocument(JsonDocument &document) {
  JsonObject gpsStatus =
      document["gps"].to<JsonObject>();

  gpsStatus["fix"] =
      gps.location.isValid() ? 1 : 0;

  gpsStatus["satellites"] =
      gps.satellites.isValid()
          ? gps.satellites.value()
          : 0;

  if (gps.hdop.isValid()) {
    gpsStatus["hdop"] = gps.hdop.hdop();
  }

  if (!gps.location.isValid()) {
    return;
  }

  JsonObject position =
      document["position"].to<JsonObject>();

  position["latitude_deg"] =
      gps.location.lat();

  position["longitude_deg"] =
      gps.location.lng();

  if (gps.speed.isValid()) {
    position["speed_mps"] =
        gps.speed.mps();
  }

  if (gps.course.isValid()) {
    position["course_deg"] =
        gps.course.deg();
  }
}

void addMotionToDocument(JsonDocument &document) {
  if (!accelerometerReady) {
    return;
  }

  sensors_event_t event;
  accelerometer.getEvent(&event);

  JsonObject motion =
      document["motion"].to<JsonObject>();

  motion["accel_x_mps2"] =
      event.acceleration.x;

  motion["accel_y_mps2"] =
      event.acceleration.y;

  motion["accel_z_mps2"] =
      event.acceleration.z;
}

void transmitTelemetry() {
  const float lm35V =
      adcToVolts(readAveragedAdc(LM35_PIN));

  const float batteryV =
      adcToVolts(readAveragedAdc(VOLTAGE_PIN)) *
      VOLTAGE_DIVIDER_RATIO;

  const float currentA =
      (
        adcToVolts(readAveragedAdc(CURRENT_PIN)) -
        ACS_ZERO_V
      ) /
      ACS_SENSITIVITY_V_PER_A;

  const bool waterDetected =
      digitalRead(WATER_PIN) == LOW;

  JsonDocument document;

  document["schema_version"] = 1;
  document["boat_id"] = BOAT_ID;
  document["sequence"] = sequenceNumber;

  document["recorded_at"] =
      "1970-01-01T00:00:00Z";

  addGpsToDocument(document);

  JsonObject power =
      document["power"].to<JsonObject>();

  power["battery_v"] = batteryV;
  power["current_a"] = currentA;
  power["power_w"] = batteryV * currentA;

  addMotionToDocument(document);

  JsonObject environment =
      document["environment"].to<JsonObject>();

  environment["electronics_temp_c"] =
      lm35V * 100.0F;

  if (!isnan(cachedAirTempC)) {
    environment["air_temp_c"] =
        cachedAirTempC;
  }

  if (!isnan(cachedHumidityPct)) {
    environment["humidity_pct"] =
        cachedHumidityPct;
  }

  environment["water_detected"] =
      waterDetected;

  JsonObject propulsion =
      document["propulsion"].to<JsonObject>();

  propulsion["pod_angle_deg"] =
      currentServoDeg;

  propulsion["target_pod_angle_deg"] =
      targetPodAngleDeg;

  propulsion["rudder_norm"] =
      rudderNormalized;

  propulsion["direction_selector_norm"] =
      directionNormalized;

  propulsion["throttle_norm"] =
      propulsionNormalized;

  propulsion["servo_pwm_us"] =
      currentServoUs;

  propulsion["esc_pwm_us"] =
      currentEscUs;

  propulsion["direction"] =
      reverseMode ? "reverse" : "forward";

  propulsion["rc_healthy"] =
      rcHealthy;

  propulsion["rudder_channel_healthy"] =
      isChannelHealthy(
        rudderChannel.lastPulseUs,
        micros()
      );

  propulsion["direction_channel_healthy"] =
      isChannelHealthy(
        directionChannel.lastPulseUs,
        micros()
      );

  propulsion["propulsion_channel_healthy"] =
      isChannelHealthy(
        propulsionChannel.lastPulseUs,
        micros()
      );

  propulsion["propulsion_armed"] =
      propulsionArmed;

  propulsion["propulsion_active"] =
      propulsionActive;

  propulsion["failsafe_active"] =
      failsafeActive;

  JsonObject status =
      document["status"].to<JsonObject>();

  JsonArray alarms =
      status["alarms"].to<JsonArray>();

  const bool batteryCritical =
      batteryV < CRITICAL_BATTERY_V;

  const bool critical =
      waterDetected ||
      batteryCritical ||
      !rcHealthy;

  status["severity"] =
      critical ? "critical" : "ok";

  if (waterDetected) {
    alarms.add("WATER_DETECTED");
  }

  if (batteryCritical) {
    alarms.add("BATTERY_CRITICAL");
  }

  if (!rcHealthy) {
    alarms.add("RC_SIGNAL_LOST");
  }

  if (!accelerometerReady) {
    alarms.add("ADXL345_UNAVAILABLE");
  }

  uint8_t payload[FRAME_MAX_PAYLOAD_BYTES];

  const size_t payloadLength =
      serializeJson(
        document,
        payload,
        sizeof(payload)
      );

  if (payloadLength == 0 ||
      payloadLength >= sizeof(payload) ||
      payloadLength > 0xFFFF) {

    return;
  }

  writeMinervaTelemetryFrame(
    payload,
    static_cast<uint16_t>(payloadLength),
    sequenceNumber,
    millis()
  );

  ++sequenceNumber;
}

// ============================================================================
// SETUP E LOOP
// ============================================================================

void setup() {
  pinMode(RUDDER_INPUT_PIN, INPUT);
  pinMode(DIRECTION_INPUT_PIN, INPUT);
  pinMode(PROPULSION_INPUT_PIN, INPUT);
  pinMode(WATER_PIN, INPUT_PULLUP);

  // USB exclusivamente para a Raspberry.
  // Nao use Serial.print de debug neste firmware.
  Serial.begin(115200);

  // GPS NEO-6M movido para Serial2:
  // RX2=D17 recebe o TX do GPS; TX2=D16 e opcional.
  // D18 fica livre para o CH2 da propulsao.
  Serial2.begin(9600);

  Wire.begin();
  dht.begin();

  accelerometerReady =
      accelerometer.begin();

  if (accelerometerReady) {
    accelerometer.setRange(
      ADXL345_RANGE_16_G
    );
  }

  azimuthServo.attach(
    AZIMUTH_SERVO_PIN,
    SERVO_MIN_US,
    SERVO_MAX_US
  );

  esc.attach(
    ESC_OUTPUT_PIN,
    RC_MIN_US,
    RC_MAX_US
  );

  reverseMode = false;
  currentServoDeg = POD_FORWARD_CENTER_DEG;
  targetPodAngleDeg = POD_FORWARD_CENTER_DEG;

  currentServoUs =
      servoDegreesToMicros(currentServoDeg);

  azimuthServo.writeMicroseconds(
    currentServoUs
  );

  currentEscUs = ESC_ARM_US;
  esc.writeMicroseconds(currentEscUs);

  delay(ESC_ARM_TIME_MS);

  resetPropulsionControl();

  attachInterrupt(
    digitalPinToInterrupt(RUDDER_INPUT_PIN),
    onRudderChange,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(DIRECTION_INPUT_PIN),
    onDirectionChange,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(PROPULSION_INPUT_PIN),
    onPropulsionChange,
    CHANGE
  );

  const uint32_t nowMs = millis();

  lastControlMs = nowMs;
  lastTelemetryMs = nowMs;
  lastDhtMs = nowMs - DHT_INTERVAL_MS;
}

void loop() {
  updateGps();

  const uint32_t nowMs = millis();

  updateDht(nowMs);

  if (nowMs - lastControlMs >=
      CONTROL_INTERVAL_MS) {

    lastControlMs = nowMs;
    updateAzimuthControl();
  }

  if (nowMs - lastTelemetryMs >=
      TELEMETRY_INTERVAL_MS) {

    lastTelemetryMs = nowMs;
    transmitTelemetry();
  }
}