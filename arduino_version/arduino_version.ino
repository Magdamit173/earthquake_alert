#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define I2C_SDA_GPIO 4
#define I2C_SCL_GPIO 5
#define BUZZER_GPIO 10

#define ADXL345_ADDRESS 0x53
#define ADXL345_REG_DEVID 0x00
#define ADXL345_REG_BWRATE 0x2C
#define ADXL345_REG_DATAFMT 0x31
#define ADXL345_REG_POWERCTL 0x2D
#define ADXL345_REG_DATAX0 0x32
#define ADXL345_DEVICE_ID 0xE5

#define ADXL345_FULL_RES 0x08
#define ADXL345_RANGE_16G 0x03
#define ADXL345_DATAFMT_VALUE (ADXL345_FULL_RES | ADXL345_RANGE_16G)
#define ADXL345_BWRATE_100HZ 0x0A
#define ADXL345_POWERCTL_MEASURE 0x08
#define ADXL345_G_PER_LSB 0.0039f

#define SAMPLE_RATE_HZ 100
#define SAMPLE_PERIOD_MS 10
#define SAMPLE_PERIOD_US 10000UL

#define CALIBRATION_SECONDS 5
#define CALIBRATION_SAMPLES (CALIBRATION_SECONDS * SAMPLE_RATE_HZ)
#define CALIBRATION_SETTLE_SAMPLES 100
#define CALIBRATION_MAX_NOISE_G 0.02f

#define HP_ALPHA 0.9695f
#define LP_ALPHA 0.4852f

#define DETECTION_NOISE_MULTIPLIER 8.0f
#define MIN_DYNAMIC_G 0.035f

#define START_CONSECUTIVE_SAMPLES 5
#define VALIDATION_MS 1000
#define VALIDATION_SAMPLES (VALIDATION_MS / SAMPLE_PERIOD_MS)
#define VALIDATION_MIN_ACTIVE_RATIO 0.45f
#define VALIDATION_STRONG_MULTIPLIER 2.0f
#define VALIDATION_MIN_STRONG_FIRST 1
#define VALIDATION_MIN_STRONG_SECOND 1
#define VALIDATION_MIN_PEAK_MULTIPLIER 2.0f
#define VALIDATION_MAX_QUIET_SAMPLES 30

#define EVENT_COOLDOWN_MS 10000UL

#define CAPTURE_RATE_HZ 10
#define CAPTURE_DECIMATION (SAMPLE_RATE_HZ / CAPTURE_RATE_HZ)
#define CAPTURE_PRE_SECONDS 2
#define CAPTURE_POST_SECONDS 3
#define CAPTURE_TOTAL_SECONDS (CAPTURE_PRE_SECONDS + CAPTURE_POST_SECONDS)
#define CAPTURE_PRE_SAMPLES (CAPTURE_PRE_SECONDS * CAPTURE_RATE_HZ)
#define CAPTURE_POST_SAMPLES (CAPTURE_POST_SECONDS * CAPTURE_RATE_HZ)
#define CAPTURE_TOTAL_SAMPLES (CAPTURE_PRE_SAMPLES + CAPTURE_POST_SAMPLES)

#define WAVEFORM_SCALE 1000

#define ASSUMED_SOURCE_DISTANCE_KM 10.0f
#define NODE_ID 4
#define JSON_BUFFER_SIZE 4096

#define SHOW_VALIDATION 1
#define SHOW_STATUS 1
#define STATUS_INTERVAL_MS 2000UL

#define BUZZER_BEEP_COUNT 3
#define BUZZER_ON_MS 250
#define BUZZER_OFF_MS 180

#define LEVEL_MILD 1
#define LEVEL_MODERATE 2
#define LEVEL_STRONG 3
#define LEVEL_VERY_STRONG 4


struct EventStats {
  uint16_t sequence;
  float peakVibration;
  float peakPgaH;
  float peakPgaV;
  float peakX;
  float peakY;
  float peakZ;
  float peakPgvH;
  float peakPgvV;
  float vx;
  float vy;
  float vz;
  float cavGs;
  float rmsSumSq;
  uint32_t rmsCount;
  unsigned long onsetMs;
  unsigned long lastActiveMs;
};

enum DetectorState {
  NORMAL = 0,
  CANDIDATE,
  CAPTURE
};

static float gravityX = 0.0f;
static float gravityY = 0.0f;
static float gravityZ = 0.0f;
static float noiseRms = 0.0f;
static int verticalAxis = 2;

static float hpPrevX = 0.0f;
static float hpPrevY = 0.0f;
static float hpPrevZ = 0.0f;
static float hpOutX = 0.0f;
static float hpOutY = 0.0f;
static float hpOutZ = 0.0f;

static float lpX = 0.0f;
static float lpY = 0.0f;
static float lpZ = 0.0f;
static bool filterReady = false;

static float preH[CAPTURE_PRE_SAMPLES];
static float preV[CAPTURE_PRE_SAMPLES];
static int preIndex = 0;
static int preCount = 0;

static float captureH[CAPTURE_TOTAL_SAMPLES];
static float captureV[CAPTURE_TOTAL_SAMPLES];

static EventStats eventStats;
static uint16_t eventSequence = 0;
static DetectorState detectorState = NORMAL;

static int candidateSamples = 0;
static int candidateActive = 0;
static int candidateStrongFirst = 0;
static int candidateStrongSecond = 0;
static int candidateQuiet = 0;
static int candidateMaxQuiet = 0;
static float candidatePeakVibration = 0.0f;
static float candidatePeakPgaH = 0.0f;
static unsigned long candidateStartMs = 0;
static unsigned long candidateLastActiveMs = 0;

static int captureCount = 0;
static int captureDivider = 0;
static int onsetStreak = 0;
static unsigned long lastEventMs = 0;
static unsigned long nextSampleUs = 0;

static bool buzzerActive = false;
static int buzzerBeepIndex = 0;
static unsigned long buzzerTimerMs = 0;
static bool buzzerPhaseOn = false;

static unsigned long statI2cFail = 0;


static float magnitude3(float x, float y, float z)
{
  return sqrtf(x * x + y * y + z * z);
}

static float horizontalMagnitude(float x, float y, float z)
{
  if (verticalAxis == 0) return sqrtf(y * y + z * z);
  if (verticalAxis == 1) return sqrtf(x * x + z * z);
  return sqrtf(x * x + y * y);
}

static float verticalValue(float x, float y, float z)
{
  if (verticalAxis == 0) return fabsf(x);
  if (verticalAxis == 1) return fabsf(y);
  return fabsf(z);
}

static void filterReset()
{
  hpPrevX = 0.0f;
  hpPrevY = 0.0f;
  hpPrevZ = 0.0f;
  hpOutX = 0.0f;
  hpOutY = 0.0f;
  hpOutZ = 0.0f;
  lpX = 0.0f;
  lpY = 0.0f;
  lpZ = 0.0f;
  filterReady = false;
}

static void filterUpdate(
  float x,
  float y,
  float z,
  float &fx,
  float &fy,
  float &fz
)
{
  if (!filterReady) {
    hpPrevX = x;
    hpPrevY = y;
    hpPrevZ = z;
    hpOutX = 0.0f;
    hpOutY = 0.0f;
    hpOutZ = 0.0f;
    filterReady = true;
  } else {
    hpOutX = HP_ALPHA * (hpOutX + x - hpPrevX);
    hpOutY = HP_ALPHA * (hpOutY + y - hpPrevY);
    hpOutZ = HP_ALPHA * (hpOutZ + z - hpPrevZ);

    hpPrevX = x;
    hpPrevY = y;
    hpPrevZ = z;
  }

  lpX += LP_ALPHA * (hpOutX - lpX);
  lpY += LP_ALPHA * (hpOutY - lpY);
  lpZ += LP_ALPHA * (hpOutZ - lpZ);

  fx = lpX;
  fy = lpY;
  fz = lpZ;
}

static void prebufferReset()
{
  memset(preH, 0, sizeof(preH));
  memset(preV, 0, sizeof(preV));
  preIndex = 0;
  preCount = 0;
}

static void prebufferAdd(float h, float v)
{
  preH[preIndex] = h;
  preV[preIndex] = v;
  preIndex = (preIndex + 1) % CAPTURE_PRE_SAMPLES;

  if (preCount < CAPTURE_PRE_SAMPLES) {
    preCount++;
  }
}

static void prebufferCopy(float *h, float *v)
{
  int missing = CAPTURE_PRE_SAMPLES - preCount;

  for (int i = 0; i < missing; i++) {
    h[i] = 0.0f;
    v[i] = 0.0f;
  }

  for (int i = missing; i < CAPTURE_PRE_SAMPLES; i++) {
    int src = (preIndex + i - missing) % CAPTURE_PRE_SAMPLES;
    h[i] = preH[src];
    v[i] = preV[src];
  }
}

static float detectionThreshold()
{
  float threshold = noiseRms * DETECTION_NOISE_MULTIPLIER;

  if (threshold < MIN_DYNAMIC_G) {
    threshold = MIN_DYNAMIC_G;
  }

  return threshold;
}

static int levelFromPga(float pga)
{
  if (pga >= 0.30f) return LEVEL_VERY_STRONG;
  if (pga >= 0.15f) return LEVEL_STRONG;
  if (pga >= 0.05f) return LEVEL_MODERATE;
  return LEVEL_MILD;
}

static const char *levelString(int level)
{
  switch (level) {
    case LEVEL_MILD: return "MILD";
    case LEVEL_MODERATE: return "MODERATE";
    case LEVEL_STRONG: return "STRONG";
    case LEVEL_VERY_STRONG: return "VERY STRONG";
    default: return "UNKNOWN";
  }
}

static const char *levelJson(int level)
{
  switch (level) {
    case LEVEL_MILD: return "mild";
    case LEVEL_MODERATE: return "moderate";
    case LEVEL_STRONG: return "strong";
    case LEVEL_VERY_STRONG: return "very_strong";
    default: return "unknown";
  }
}

static int estimateIntensity(float pgaHG, float pgvH)
{
  if (pgaHG <= 0.0f) return 1;

  float pgaCms2 = pgaHG * 980.665f;
  float logPga = log10f(pgaCms2 > 0.001f ? pgaCms2 : 0.001f);

  float mmiPga =
    (logPga <= 1.57f)
      ? (1.78f + 1.55f * logPga)
      : (-1.60f + 3.70f * logPga);

  float mmi = mmiPga;

  if (pgvH > 0.001f) {
    float logPgv = log10f(pgvH);

    float mmiPgv =
      (logPgv <= 0.53f)
        ? (3.78f + 1.47f * logPgv)
        : (2.89f + 3.16f * logPgv);

    if (mmiPga < 5.0f) {
      mmi = mmiPga;
    } else if (mmiPga >= 7.0f) {
      mmi = mmiPgv;
    } else {
      float w = (mmiPga - 5.0f) / 2.0f;
      mmi = mmiPga * (1.0f - w) + mmiPgv * w;
    }
  }

  int value = (int)lroundf(mmi);

  if (value < 1) value = 1;
  if (value > 8) value = 8;

  return value;
}

static const char *intensityString(int intensity)
{
  switch (intensity) {
    case 1: return "I";
    case 2: return "II";
    case 3: return "III";
    case 4: return "IV";
    case 5: return "V";
    case 6: return "VI";
    case 7: return "VII";
    case 8: return "VIII";
    default: return "UNKNOWN";
  }
}

static float estimateMagnitude(float pgaHG, float distanceKm)
{
  if (pgaHG <= 0.000001f || distanceKm < 0.0f) {
    return 0.0f;
  }

  float magnitude =
    (logf(pgaHG) +
     logf(distanceKm + 7.28f) +
     2.501f) / 0.623f;

  if (magnitude < 0.0f) magnitude = 0.0f;
  if (magnitude > 9.9f) magnitude = 9.9f;

  return magnitude;
}

static void velocityUpdate(
  float ax,
  float ay,
  float az,
  float &vx,
  float &vy,
  float &vz
)
{
  const float leak = 0.995f;
  const float dtScale = 980.665f * 0.01f;

  vx = leak * vx + ax * dtScale;
  vy = leak * vy + ay * dtScale;
  vz = leak * vz + az * dtScale;

  if (fabsf(vx) < 0.02f) vx = 0.0f;
  if (fabsf(vy) < 0.02f) vy = 0.0f;
  if (fabsf(vz) < 0.02f) vz = 0.0f;
}

static float velocityHorizontal(float vx, float vy, float vz)
{
  return horizontalMagnitude(vx, vy, vz);
}

static float velocityVertical(float vx, float vy, float vz)
{
  if (verticalAxis == 0) return fabsf(vx);
  if (verticalAxis == 1) return fabsf(vy);
  return fabsf(vz);
}

static bool adxlWrite(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool adxlRead(uint8_t reg, uint8_t *data, size_t length)
{
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom((int)ADXL345_ADDRESS, (int)length);

  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    if (!Wire.available()) {
      return false;
    }

    data[i] = Wire.read();
  }

  return true;
}

static bool adxlInit()
{
  for (int attempt = 0; attempt < 10; attempt++) {
    uint8_t id = 0;

    bool ok =
      adxlRead(ADXL345_REG_DEVID, &id, 1) &&
      id == ADXL345_DEVICE_ID &&
      adxlWrite(ADXL345_REG_DATAFMT, ADXL345_DATAFMT_VALUE) &&
      adxlWrite(ADXL345_REG_BWRATE, ADXL345_BWRATE_100HZ) &&
      adxlWrite(ADXL345_REG_POWERCTL, ADXL345_POWERCTL_MEASURE);

    if (ok) {
      delay(50);
      return true;
    }

    delay(100);
  }

  return false;
}

static bool adxlReadG(float &x, float &y, float &z)
{
  uint8_t data[6];

  if (!adxlRead(ADXL345_REG_DATAX0, data, sizeof(data))) {
    return false;
  }

  int16_t rawX = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
  int16_t rawZ = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

  x = rawX * ADXL345_G_PER_LSB;
  y = rawY * ADXL345_G_PER_LSB;
  z = rawZ * ADXL345_G_PER_LSB;

  return true;
}

static void calibrationBeep(int count, int onMs = 120, int offMs = 120)
{
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_GPIO, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_GPIO, LOW);

    if (i + 1 < count) {
      delay(offMs);
    }
  }
}

static bool calibrateSensor()
{
  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float x;
    float y;
    float z;

    if (!adxlReadG(x, y, z)) {
      Serial.println("CALIBRATION: sensor read failed");
      return false;
    }

    sumX += x;
    sumY += y;
    sumZ += z;

    delay(SAMPLE_PERIOD_MS);
  }

  gravityX = sumX / CALIBRATION_SAMPLES;
  gravityY = sumY / CALIBRATION_SAMPLES;
  gravityZ = sumZ / CALIBRATION_SAMPLES;

  float absX = fabsf(gravityX);
  float absY = fabsf(gravityY);
  float absZ = fabsf(gravityZ);

  if (absX >= absY && absX >= absZ) {
    verticalAxis = 0;
  } else if (absY >= absZ) {
    verticalAxis = 1;
  } else {
    verticalAxis = 2;
  }

  float g = magnitude3(gravityX, gravityY, gravityZ);

  if (g < 0.8f || g > 1.2f) {
    Serial.printf("CALIBRATION: gravity out of range (%.3f g)\n", g);
    return false;
  }

  filterReset();

  Serial.println("PHASE 1 COMPLETE");
  Serial.println("PHASE 2: NOISE CALIBRATION (5 s)");
  calibrationBeep(3, 100, 100);

  float noiseSum = 0.0f;
  int noiseCount = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float x;
    float y;
    float z;
    float fx;
    float fy;
    float fz;

    if (!adxlReadG(x, y, z)) {
      Serial.println("CALIBRATION: sensor read failed");
      return false;
    }

    filterUpdate(
      x - gravityX,
      y - gravityY,
      z - gravityZ,
      fx,
      fy,
      fz
    );

    if (i >= CALIBRATION_SETTLE_SAMPLES) {
      float motion = magnitude3(fx, fy, fz);
      noiseSum += motion * motion;
      noiseCount++;
    }

    delay(SAMPLE_PERIOD_MS);
  }

  noiseRms = sqrtf(
    noiseSum / (noiseCount > 0 ? noiseCount : 1)
  );

  filterReset();
  prebufferReset();

  return true;
}

static void fatalSensorError(const char *message)
{
  digitalWrite(BUZZER_GPIO, LOW);
  Serial.println(message);

  while (true) {
    delay(1000);
  }
}

static void startBuzzerAlert()
{
  if (buzzerActive) {
    return;
  }

  buzzerActive = true;
  buzzerBeepIndex = 0;
  buzzerPhaseOn = true;
  buzzerTimerMs = millis();

  digitalWrite(BUZZER_GPIO, HIGH);
}

static void serviceBuzzer()
{
  if (!buzzerActive) {
    return;
  }

  unsigned long now = millis();

  if (buzzerPhaseOn) {
    if (now - buzzerTimerMs >= BUZZER_ON_MS) {
      digitalWrite(BUZZER_GPIO, LOW);
      buzzerPhaseOn = false;
      buzzerTimerMs = now;
    }

    return;
  }

  if (buzzerBeepIndex + 1 >= BUZZER_BEEP_COUNT) {
    buzzerActive = false;
    return;
  }

  if (now - buzzerTimerMs >= BUZZER_OFF_MS) {
    buzzerBeepIndex++;
    buzzerPhaseOn = true;
    buzzerTimerMs = now;

    digitalWrite(BUZZER_GPIO, HIGH);
  }
}

static bool appendJson(
  char *buffer,
  size_t capacity,
  size_t &length,
  const char *format,
  ...
)
{
  if (length >= capacity) {
    return false;
  }

  va_list args;
  va_start(args, format);

  int written =
    vsnprintf(
      buffer + length,
      capacity - length,
      format,
      args
    );

  va_end(args);

  if (written < 0) {
    return false;
  }

  if ((size_t)written >= capacity - length) {
    return false;
  }

  length += (size_t)written;
  return true;
}

static float dominantFrequency(
  const float *h,
  const float *v
)
{
  float bestFrequency = 0.0f;
  float bestPower = 0.0f;

  for (int k = 1; k <= 10; k++) {
    float frequency = 0.5f * (float)k;
    float power = 0.0f;

    for (int axis = 0; axis < 2; axis++) {
      const float *samples = axis == 0 ? h : v;
      float mean = 0.0f;

      for (int n = 0; n < CAPTURE_TOTAL_SAMPLES; n++) {
        mean += samples[n];
      }

      mean /= CAPTURE_TOTAL_SAMPLES;

      float real = 0.0f;
      float imag = 0.0f;

      for (int n = 0; n < CAPTURE_TOTAL_SAMPLES; n++) {
        float phase =
          2.0f * PI * frequency * (float)n /
          (float)CAPTURE_RATE_HZ;

        float window =
          0.5f -
          0.5f * cosf(
            2.0f * PI * (float)n /
            (float)(CAPTURE_TOTAL_SAMPLES - 1)
          );

        float sample = samples[n] - mean;

        real += sample * window * cosf(phase);
        imag -= sample * window * sinf(phase);
      }

      power += real * real + imag * imag;
    }

    if (power > bestPower) {
      bestPower = power;
      bestFrequency = frequency;
    }
  }

  return bestFrequency;
}

static bool buildJson(
  char *json,
  size_t capacity,
  size_t &length
)
{
  int intensity =
    estimateIntensity(
      eventStats.peakPgaH,
      eventStats.peakPgvH
    );

  int level =
    levelFromPga(eventStats.peakPgaH);

  float rms =
    sqrtf(
      eventStats.rmsSumSq /
      (eventStats.rmsCount > 0 ? eventStats.rmsCount : 1)
    );

  float duration =
    (float)(
      eventStats.lastActiveMs -
      eventStats.onsetMs
    ) / 1000.0f;

  float frequency =
    dominantFrequency(
      captureH,
      captureV
    );

  float magnitude =
    estimateMagnitude(
      eventStats.peakPgaH,
      ASSUMED_SOURCE_DISTANCE_KM
    );

  length = 0;

  if (!appendJson(
        json,
        capacity,
        length,
        "{\"version\":1,\"event\":\"earthquake\",\"node\":%d,\"sequence\":%u,"
        "\"t_ms\":%lu,"
        "\"sample_rate_hz\":%d,\"span_s\":%d,\"pre_s\":%d,\"post_s\":%d,"
        "\"vertical_axis\":\"%c\",\"level\":\"%s\",\"intensity\":%d,\"intensity_roman\":\"%s\","
        "\"intensity_note\":\"instrumental_mmi_estimate\","
        "\"pga_g\":%.4f,\"pga_h_g\":%.4f,\"pga_v_g\":%.4f,"
        "\"pga_h_cm_s2\":%.1f,\"pga_v_cm_s2\":%.1f,"
        "\"peak_vibration_g\":%.4f,\"peak_x_g\":%.4f,\"peak_y_g\":%.4f,\"peak_z_g\":%.4f,"
        "\"pgv_h_cm_s\":%.3f,\"pgv_v_cm_s\":%.3f,"
        "\"rms_vibration_g\":%.4f,\"cav_g_s\":%.4f,\"duration_s\":%.3f,"
        "\"dominant_hz\":%.1f,\"magnitude_est\":%.2f,"
        "\"magnitude_distance_assumed_km\":%.1f,"
        "\"magnitude_note\":\"rough_estimate\","
        "\"threshold_g\":%.5f,\"waveform_unit\":\"mg\",\"h\":[",
        NODE_ID,
        (unsigned)eventStats.sequence,
        eventStats.onsetMs,
        CAPTURE_RATE_HZ,
        CAPTURE_TOTAL_SECONDS,
        CAPTURE_PRE_SECONDS,
        CAPTURE_POST_SECONDS,
        "XYZ"[verticalAxis],
        levelJson(level),
        intensity,
        intensityString(intensity),
        eventStats.peakVibration,
        eventStats.peakPgaH,
        eventStats.peakPgaV,
        eventStats.peakPgaH * 980.665f,
        eventStats.peakPgaV * 980.665f,
        eventStats.peakVibration,
        eventStats.peakX,
        eventStats.peakY,
        eventStats.peakZ,
        eventStats.peakPgvH,
        eventStats.peakPgvV,
        rms,
        eventStats.cavGs,
        duration,
        frequency,
        magnitude,
        ASSUMED_SOURCE_DISTANCE_KM,
        detectionThreshold()
      )) {
    return false;
  }

  for (int i = 0; i < CAPTURE_TOTAL_SAMPLES; i++) {
    if (!appendJson(
          json,
          capacity,
          length,
          "%s%d",
          i == 0 ? "" : ",",
          (int)lroundf(captureH[i] * WAVEFORM_SCALE)
        )) {
      return false;
    }
  }

  if (!appendJson(
        json,
        capacity,
        length,
        "],\"v\":["
      )) {
    return false;
  }

  for (int i = 0; i < CAPTURE_TOTAL_SAMPLES; i++) {
    if (!appendJson(
          json,
          capacity,
          length,
          "%s%d",
          i == 0 ? "" : ",",
          (int)lroundf(captureV[i] * WAVEFORM_SCALE)
        )) {
      return false;
    }
  }

  if (!appendJson(
        json,
        capacity,
        length,
        "]}"
      )) {
    return false;
  }

  return true;
}

static void printJsonEvent()
{
  static char json[JSON_BUFFER_SIZE];
  size_t length = 0;

  if (!buildJson(json, sizeof(json), length)) {
    Serial.println("{\"event\":\"error\",\"error\":\"json_build_failed\"}");
    return;
  }

  Serial.println();
  Serial.println("JSON_EVENT_BEGIN");
  Serial.write((const uint8_t *)json, length);
  Serial.println();
  Serial.println("JSON_EVENT_END");
  Serial.printf("JSON_BYTES:%u\n", (unsigned)length);
}

static void printEventSummary()
{
  int intensity =
    estimateIntensity(
      eventStats.peakPgaH,
      eventStats.peakPgvH
    );

  int level =
    levelFromPga(eventStats.peakPgaH);

  float rms =
    sqrtf(
      eventStats.rmsSumSq /
      (eventStats.rmsCount > 0 ? eventStats.rmsCount : 1)
    );

  float duration =
    (float)(
      eventStats.lastActiveMs -
      eventStats.onsetMs
    ) / 1000.0f;

  float frequency =
    dominantFrequency(
      captureH,
      captureV
    );

  float magnitude =
    estimateMagnitude(
      eventStats.peakPgaH,
      ASSUMED_SOURCE_DISTANCE_KM
    );

  Serial.println();
  Serial.println("================================");
  Serial.println("EARTHQUAKE DETECTED");
  Serial.println("================================");
  Serial.printf("NODE ID:           %d\n", NODE_ID);
  Serial.printf("SEQUENCE:          %u\n", (unsigned)eventStats.sequence);
  Serial.printf("LEVEL:             %s\n", levelString(level));
  Serial.printf("INTENSITY (est):   %s\n", intensityString(intensity));
  Serial.printf("PEAK ACCEL:        %.4f g\n", eventStats.peakVibration);
  Serial.printf("PEAK H PGA:        %.4f g\n", eventStats.peakPgaH);
  Serial.printf("PEAK V PGA:        %.4f g\n", eventStats.peakPgaV);
  Serial.printf("PEAK H PGV:        %.3f cm/s\n", eventStats.peakPgvH);
  Serial.printf("PEAK V PGV:        %.3f cm/s\n", eventStats.peakPgvV);
  Serial.printf("RMS VIBRATION:     %.4f g\n", rms);
  Serial.printf("CAV:               %.4f g*s\n", eventStats.cavGs);
  Serial.printf("DURATION:          %.3f s\n", duration);
  Serial.printf("DOMINANT FREQ:     %.1f Hz\n", frequency);
  Serial.printf("MAGNITUDE (rough): %.2f\n", magnitude);
  Serial.printf("DISTANCE ASSUMED:  %.1f km\n", ASSUMED_SOURCE_DISTANCE_KM);
  Serial.println("================================");
  printJsonEvent();
}

static void printReadyInfo()
{
  Serial.println();
  Serial.println("CALIBRATION COMPLETE");
  Serial.printf("GRAVITY: X=%.4f Y=%.4f Z=%.4f g\n", gravityX, gravityY, gravityZ);
  Serial.printf("VERTICAL AXIS: %c\n", "XYZ"[verticalAxis]);
  Serial.printf("NOISE RMS: %.5f g\n", noiseRms);
  Serial.printf("THRESHOLD: %.5f g\n", detectionThreshold());
  Serial.printf("CAPTURE: %d s @ %d Hz\n", CAPTURE_TOTAL_SECONDS, CAPTURE_RATE_HZ);
  Serial.println("READY");
}

void setup()
{
  Serial.begin(115200);

  pinMode(BUZZER_GPIO, OUTPUT);
  digitalWrite(BUZZER_GPIO, LOW);

  unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 8000UL) {
    yield();
  }

  delay(300);

  Wire.begin(
    I2C_SDA_GPIO,
    I2C_SCL_GPIO
  );

  Wire.setClock(100000);

  Serial.println();
  Serial.println("================================");
  Serial.println("EARTHQUAKE ALERT SLAVE");
  Serial.println("================================");
  Serial.printf("NODE ID: %d\n", NODE_ID);

  if (!adxlInit()) {
    fatalSensorError("ADXL345 ERROR");
  }

  Serial.println("ADXL345 OK");
  Serial.println("KEEP SENSOR STILL");
  Serial.println("CALIBRATION START");
  Serial.println("PHASE 1: GRAVITY CALIBRATION (5 s)");
  calibrationBeep(1, 500, 0);
  delay(200);
  calibrationBeep(2, 120, 120);

  bool calibrated = false;

  while (!calibrated) {
    calibrated = calibrateSensor();

    if (!calibrated) {
      Serial.println("CALIBRATION RETRY");
      delay(1000);
      Serial.println("PHASE 1: GRAVITY CALIBRATION (5 s)");
    }
  }

  if (noiseRms > CALIBRATION_MAX_NOISE_G) {
    Serial.printf("WARNING: HIGH NOISE %.5f g (sensor moved?)\n", noiseRms);
  }

  lastEventMs = millis() - EVENT_COOLDOWN_MS;

  filterReset();

  Serial.println("PHASE 2 COMPLETE");
  calibrationBeep(2, 300, 150);
  printReadyInfo();

  nextSampleUs = micros();
}

void loop()
{
  serviceBuzzer();

  static bool serialWasOpen = true;
  bool serialOpen = (bool)Serial;

  if (serialOpen && !serialWasOpen && detectorState == NORMAL) {
    Serial.println();
    Serial.println("SERIAL CONNECTED");
    printReadyInfo();
    nextSampleUs = micros();
  }

  serialWasOpen = serialOpen;

  while ((long)(micros() - nextSampleUs) < 0) {
    serviceBuzzer();
    yield();
  }

  nextSampleUs += SAMPLE_PERIOD_US;

  float ax;
  float ay;
  float az;

  if (!adxlReadG(ax, ay, az)) {
    statI2cFail++;
    return;
  }

  float dx = ax - gravityX;
  float dy = ay - gravityY;
  float dz = az - gravityZ;

  float fx;
  float fy;
  float fz;

  filterUpdate(dx, dy, dz, fx, fy, fz);

  float vibration = magnitude3(fx, fy, fz);
  float pgaH = horizontalMagnitude(fx, fy, fz);
  float pgaV = verticalValue(fx, fy, fz);
  float threshold = detectionThreshold();

  bool amplitudeTrigger = vibration >= threshold;

  if (amplitudeTrigger) {
    if (onsetStreak < START_CONSECUTIVE_SAMPLES) {
      onsetStreak++;
    }
  } else {
    onsetStreak = 0;
  }

  bool onsetTrigger = onsetStreak >= START_CONSECUTIVE_SAMPLES;

  captureDivider++;

  bool captureSampleDue = false;

  if (captureDivider >= CAPTURE_DECIMATION) {
    captureDivider = 0;
    captureSampleDue = true;
  }

  if (detectorState != CAPTURE && captureSampleDue) {
    prebufferAdd(pgaH, pgaV);
  }

  if (detectorState == CAPTURE) {
    const float dt = 0.01f;

    eventStats.cavGs += vibration * dt;
    eventStats.rmsSumSq += vibration * vibration;
    eventStats.rmsCount++;

    velocityUpdate(
      fx,
      fy,
      fz,
      eventStats.vx,
      eventStats.vy,
      eventStats.vz
    );

    float pgvH =
      velocityHorizontal(
        eventStats.vx,
        eventStats.vy,
        eventStats.vz
      );

    float pgvV =
      velocityVertical(
        eventStats.vx,
        eventStats.vy,
        eventStats.vz
      );

    if (vibration > eventStats.peakVibration) {
      eventStats.peakVibration = vibration;
    }

    if (pgaH > eventStats.peakPgaH) {
      eventStats.peakPgaH = pgaH;
    }

    if (pgaV > eventStats.peakPgaV) {
      eventStats.peakPgaV = pgaV;
    }

    if (fabsf(fx) > eventStats.peakX) {
      eventStats.peakX = fabsf(fx);
    }

    if (fabsf(fy) > eventStats.peakY) {
      eventStats.peakY = fabsf(fy);
    }

    if (fabsf(fz) > eventStats.peakZ) {
      eventStats.peakZ = fabsf(fz);
    }

    if (pgvH > eventStats.peakPgvH) {
      eventStats.peakPgvH = pgvH;
    }

    if (pgvV > eventStats.peakPgvV) {
      eventStats.peakPgvV = pgvV;
    }

    if (amplitudeTrigger) {
      eventStats.lastActiveMs = millis();
    }

    if (captureSampleDue && captureCount < CAPTURE_TOTAL_SAMPLES) {
      captureH[captureCount] = pgaH;
      captureV[captureCount] = pgaV;
      captureCount++;
    }

    if (captureCount >= CAPTURE_TOTAL_SAMPLES) {
      printEventSummary();

      detectorState = NORMAL;
      lastEventMs = millis();
      captureCount = 0;
      captureDivider = 0;
      onsetStreak = 0;

      memset(&eventStats, 0, sizeof(eventStats));

      prebufferReset();
      filterReset();
      nextSampleUs = micros();
    }
  } else if (detectorState == NORMAL) {
    if (onsetTrigger &&
        millis() - lastEventMs >= EVENT_COOLDOWN_MS &&
        preCount >= CAPTURE_PRE_SAMPLES) {

      detectorState = CANDIDATE;

      candidateSamples = 0;
      candidateActive = 0;
      candidateStrongFirst = 0;
      candidateStrongSecond = 0;
      candidateQuiet = 0;
      candidateMaxQuiet = 0;
      candidatePeakVibration = vibration;
      candidatePeakPgaH = pgaH;
      candidateStartMs = millis();
      candidateLastActiveMs = candidateStartMs;

      onsetStreak = 0;

#if SHOW_VALIDATION
      Serial.println("SHAKE DETECTED, VALIDATING (1 s)...");
#endif
    }
  }

  if (detectorState == CANDIDATE) {
    candidateSamples++;

    if (vibration >= threshold * 0.6f) {
      candidateActive++;
      candidateQuiet = 0;
      candidateLastActiveMs = millis();
    } else {
      candidateQuiet++;

      if (candidateQuiet > candidateMaxQuiet) {
        candidateMaxQuiet = candidateQuiet;
      }
    }

    if (vibration >= threshold * VALIDATION_STRONG_MULTIPLIER) {
      if (candidateSamples <= VALIDATION_SAMPLES / 2) {
        candidateStrongFirst++;
      } else {
        candidateStrongSecond++;
      }
    }

    if (vibration > candidatePeakVibration) {
      candidatePeakVibration = vibration;
    }

    if (pgaH > candidatePeakPgaH) {
      candidatePeakPgaH = pgaH;
    }

    if (candidateSamples >= VALIDATION_SAMPLES) {
      float activeRatio =
        (float)candidateActive /
        (float)VALIDATION_SAMPLES;

      bool enoughActivity =
        activeRatio >= VALIDATION_MIN_ACTIVE_RATIO;

      bool strongEnough =
        candidateStrongFirst >= VALIDATION_MIN_STRONG_FIRST &&
        candidateStrongSecond >= VALIDATION_MIN_STRONG_SECOND;

      bool peakEnough =
        candidatePeakVibration >=
        threshold * VALIDATION_MIN_PEAK_MULTIPLIER;

      bool gapOkay =
        candidateMaxQuiet <= VALIDATION_MAX_QUIET_SAMPLES;

      bool confirmed =
        enoughActivity &&
        strongEnough &&
        peakEnough &&
        gapOkay;

#if SHOW_VALIDATION
      Serial.printf(
        "VALIDATION | active=%.2f | strong=%d/%d | peak=%.4f | quiet=%d | %s\n",
        activeRatio,
        candidateStrongFirst,
        candidateStrongSecond,
        candidatePeakVibration,
        candidateMaxQuiet,
        confirmed ? "CONFIRMED" : "DISCARDED"
      );
#endif

      if (confirmed) {
        detectorState = CAPTURE;

        memset(&eventStats, 0, sizeof(eventStats));

        eventStats.sequence = ++eventSequence;
        eventStats.onsetMs = candidateStartMs;
        eventStats.lastActiveMs = candidateLastActiveMs;
        eventStats.peakVibration = candidatePeakVibration;
        eventStats.peakPgaH = candidatePeakPgaH;

        prebufferCopy(captureH, captureV);

        captureCount = CAPTURE_PRE_SAMPLES;
        captureDivider = 0;

        startBuzzerAlert();

        Serial.println("EARTHQUAKE CONFIRMED");
        Serial.println("CAPTURING 5 SECOND EVENT WINDOW");
        nextSampleUs = micros();
      } else {
        detectorState = NORMAL;
        onsetStreak = 0;
      }
    }
  }

#if SHOW_STATUS
  static unsigned long lastStatusMs = 0;

  if (detectorState == NORMAL &&
      millis() - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = millis();
    Serial.printf(
      "MONITORING | vib=%.4f g | threshold=%.4f g | i2c_fail=%lu\n",
      vibration,
      threshold,
      statI2cFail
    );
  }
#endif
}
