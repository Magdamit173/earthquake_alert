#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <time.h>

#define I2C_SDA_GPIO 4
#define I2C_SCL_GPIO 5
#define BUZZER_GPIO 10

#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#define ADXL345_ADDRESS 0x53
#define ADXL345_REG_DEVID 0x00
#define ADXL345_REG_BWRATE 0x2C
#define ADXL345_REG_DATAFMT 0x31
#define ADXL345_REG_POWERCTL 0x2D
#define ADXL345_REG_DATAX0 0x32
#define ADXL345_DEVICE_ID 0xE5

#define SAMPLE_RATE_HZ 100
#define SAMPLE_PERIOD_MS 10
#define CALIBRATION_SECONDS 5

#define FILTER_ALPHA 0.01f

#define EARTHQUAKE_THRESHOLD_G 0.08f
#define REQUIRED_HITS 8
#define HIT_WINDOW 10

#define EVENT_ANALYSIS_MS 3000
#define EVENT_COOLDOWN_MS 10000

#define ASSUMED_SOURCE_DISTANCE_KM 10.0f

#define ADXL345_G_PER_LSB 0.0039f

#define NODE_ID 4
#define NODE_LAT 14.676000f
#define NODE_LNG 121.043700f
#define ESPNOW_CHANNEL 1

#define HAZARD_NONE 0
#define HAZARD_EARTHQUAKE 5

#define HEARTBEAT_MS 1000
#define ALERT_HOLD_MS 30000

typedef struct {
  int nodeID;
  float locationLat;
  float locationLng;
  int hazardType;
  int severityLevel;
} disaster_message_t;

typedef struct {
  int nodeID;
} ack_message_t;

uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

volatile bool hubAcked = false;

float lowpassX = 0.0f;
float lowpassY = 0.0f;
float lowpassZ = 0.0f;

int alertSeverity = 0;
unsigned long alertUntil = 0;

unsigned long lastHeartbeat = 0;
unsigned long lastEvent = 0;

bool eventActive = false;

unsigned long eventStart = 0;

time_t eventEpoch = 0;

float peakVibration = 0.0f;
float peakAcceleration = 0.0f;
float peakPGA = 0.0f;

bool ackReported = false;

void writeRegister(
  uint8_t reg,
  uint8_t value
) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readRegisters(
  uint8_t reg,
  uint8_t *data,
  uint8_t length
) {
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received =
    Wire.requestFrom(
      ADXL345_ADDRESS,
      length
    );

  if (received != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    data[i] = Wire.read();
  }

  return true;
}

bool adxlInit() {
  uint8_t deviceID = 0;

  if (
    !readRegisters(
      ADXL345_REG_DEVID,
      &deviceID,
      1
    )
  ) {
    return false;
  }

  if (deviceID != ADXL345_DEVICE_ID) {
    return false;
  }

  writeRegister(
    ADXL345_REG_DATAFMT,
    0x08
  );

  writeRegister(
    ADXL345_REG_BWRATE,
    0x0A
  );

  writeRegister(
    ADXL345_REG_POWERCTL,
    0x08
  );

  return true;
}

bool readRaw(
  int16_t &x,
  int16_t &y,
  int16_t &z
) {
  uint8_t data[6];

  if (
    !readRegisters(
      ADXL345_REG_DATAX0,
      data,
      6
    )
  ) {
    return false;
  }

  x =
    (int16_t)(
      ((uint16_t)data[1] << 8) |
      data[0]
    );

  y =
    (int16_t)(
      ((uint16_t)data[3] << 8) |
      data[2]
    );

  z =
    (int16_t)(
      ((uint16_t)data[5] << 8) |
      data[4]
    );

  return true;
}

void buzzerInit() {
  pinMode(
    BUZZER_GPIO,
    OUTPUT
  );

  digitalWrite(
    BUZZER_GPIO,
    LOW
  );
}

void buzzerOn() {
  digitalWrite(
    BUZZER_GPIO,
    HIGH
  );
}

void buzzerOff() {
  digitalWrite(
    BUZZER_GPIO,
    LOW
  );
}

void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  if (
    len != sizeof(ack_message_t)
  ) {
    return;
  }

  ack_message_t ack;

  memcpy(
    &ack,
    data,
    sizeof(ack)
  );

  if (
    ack.nodeID == NODE_ID
  ) {
    hubAcked = true;
  }
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  unsigned long start =
    millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < 15000
  ) {
    delay(100);
  }

  return WiFi.status() ==
         WL_CONNECTED;
}

bool syncTime() {
  configTime(
    0,
    0,
    "pool.ntp.org"
  );

  unsigned long start =
    millis();

  while (
    millis() - start < 15000
  ) {
    time_t now =
      time(nullptr);

    if (
      now >= 1700000000
    ) {
      return true;
    }

    delay(100);
  }

  return false;
}

bool espNowStart() {
  WiFi.disconnect(true);

  delay(500);

  WiFi.mode(WIFI_STA);

  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  uint8_t primary;
  wifi_second_chan_t second;

  esp_wifi_get_channel(
    &primary,
    &second
  );

  if (
    primary != ESPNOW_CHANNEL
  ) {
    return false;
  }

  if (
    esp_now_init() != ESP_OK
  ) {
    return false;
  }

  esp_now_register_recv_cb(
    onDataRecv
  );

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    broadcastAddress,
    6
  );

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (
    esp_now_add_peer(
      &peerInfo
    ) != ESP_OK
  ) {
    return false;
  }

  return true;
}

void hubSend(
  int hazard,
  int severity
) {
  disaster_message_t message;

  message.nodeID =
    NODE_ID;

  message.locationLat =
    NODE_LAT;

  message.locationLng =
    NODE_LNG;

  message.hazardType =
    hazard;

  message.severityLevel =
    severity;

  esp_now_send(
    broadcastAddress,
    (uint8_t *)&message,
    sizeof(message)
  );
}

int severityFromMMI(
  int mmi
) {
  if (mmi >= 8) {
    return 3;
  }

  if (mmi >= 6) {
    return 2;
  }

  return 1;
}

bool calibrateSensor() {
  const int samples =
    CALIBRATION_SECONDS *
    SAMPLE_RATE_HZ;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  for (
    int i = 0;
    i < samples;
    i++
  ) {
    int16_t x;
    int16_t y;
    int16_t z;

    if (
      !readRaw(
        x,
        y,
        z
      )
    ) {
      return false;
    }

    sumX +=
      x *
      ADXL345_G_PER_LSB;

    sumY +=
      y *
      ADXL345_G_PER_LSB;

    sumZ +=
      z *
      ADXL345_G_PER_LSB;

    delay(
      SAMPLE_PERIOD_MS
    );
  }

  lowpassX =
    sumX /
    samples;

  lowpassY =
    sumY /
    samples;

  lowpassZ =
    sumZ /
    samples;

  return true;
}

float calculateVibration(
  float ax,
  float ay,
  float az
) {
  lowpassX +=
    FILTER_ALPHA *
    (ax - lowpassX);

  lowpassY +=
    FILTER_ALPHA *
    (ay - lowpassY);

  lowpassZ +=
    FILTER_ALPHA *
    (az - lowpassZ);

  float dx =
    ax - lowpassX;

  float dy =
    ay - lowpassY;

  float dz =
    az - lowpassZ;

  return sqrtf(
    dx * dx +
    dy * dy +
    dz * dz
  );
}

bool earthquakeDetected(
  float vibration
) {
  static int hitCount = 0;
  static int sampleCount = 0;

  sampleCount++;

  if (
    vibration >=
    EARTHQUAKE_THRESHOLD_G
  ) {
    hitCount++;
  }

  if (
    sampleCount >= HIT_WINDOW
  ) {
    bool detected =
      hitCount >= REQUIRED_HITS;

    sampleCount = 0;
    hitCount = 0;

    return detected;
  }

  return false;
}

float estimateMagnitude(
  float pgaG
) {
  if (
    pgaG <= 0.000001f
  ) {
    return 0.0f;
  }

  float magnitude =
    (
      logf(pgaG) +
      logf(
        ASSUMED_SOURCE_DISTANCE_KM +
        7.28f
      ) +
      2.501f
    ) /
    0.623f;

  if (magnitude < 0.0f) {
    magnitude = 0.0f;
  }

  if (magnitude > 9.9f) {
    magnitude = 9.9f;
  }

  return magnitude;
}

int estimateIntensity(
  float pgaG
) {
  if (
    pgaG <= 0.0f
  ) {
    return 0;
  }

  float intensity =
    3.66f *
    log10f(
      pgaG * 100.0f
    ) +
    1.99f;

  int mmi =
    (int)lroundf(
      intensity
    );

  if (mmi < 5) {
    mmi = 5;
  }

  if (mmi > 8) {
    mmi = 8;
  }

  return mmi;
}

const char *mmiString(
  int intensity
) {
  switch (intensity) {

    case 1:
      return "I";

    case 2:
      return "II";

    case 3:
      return "III";

    case 4:
      return "IV";

    case 5:
      return "V";

    case 6:
      return "VI";

    case 7:
      return "VII";

    case 8:
      return "VIII";

    case 9:
      return "IX";

    case 10:
      return "X";

    default:
      return "UNKNOWN";
  }
}

void haltSystem() {
  while (true) {
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(
    I2C_SDA_GPIO,
    I2C_SCL_GPIO
  );

  Wire.setClock(100000);

  buzzerInit();

  if (!adxlInit()) {
    haltSystem();
  }

  bool connected =
    connectWiFi();

  if (connected) {
    syncTime();
  }

  if (!espNowStart()) {
    haltSystem();
  }

  if (!calibrateSensor()) {
    haltSystem();
  }

  lastEvent =
    millis() -
    EVENT_COOLDOWN_MS;
}

void loop() {
  unsigned long now =
    millis();

  if (
    alertSeverity &&
    now > alertUntil
  ) {
    alertSeverity = 0;
  }

  if (
    now - lastHeartbeat >=
    HEARTBEAT_MS
  ) {
    hubSend(
      alertSeverity
        ? HAZARD_EARTHQUAKE
        : HAZARD_NONE,
      alertSeverity
    );

    lastHeartbeat = now;
  }

  if (
    hubAcked &&
    !ackReported
  ) {
    Serial.println(
      "HUB ACK: help is on the way"
    );

    ackReported = true;
  }

  int16_t rawX;
  int16_t rawY;
  int16_t rawZ;

  if (
    !readRaw(
      rawX,
      rawY,
      rawZ
    )
  ) {
    delay(
      SAMPLE_PERIOD_MS
    );

    return;
  }

  float ax =
    rawX *
    ADXL345_G_PER_LSB;

  float ay =
    rawY *
    ADXL345_G_PER_LSB;

  float az =
    rawZ *
    ADXL345_G_PER_LSB;

  float vibration =
    calculateVibration(
      ax,
      ay,
      az
    );

  float totalAcceleration =
    sqrtf(
      ax * ax +
      ay * ay +
      az * az
    );

  if (!eventActive) {

    if (
      earthquakeDetected(
        vibration
      ) &&
      (
        now - lastEvent >=
        EVENT_COOLDOWN_MS
      )
    ) {

      eventActive = true;

      eventStart =
        now;

      eventEpoch =
        time(nullptr);

      peakVibration =
        vibration;

      peakAcceleration =
        totalAcceleration;

      peakPGA =
        vibration;

      buzzerOn();

      alertSeverity = 2;

      alertUntil =
        now +
        EVENT_ANALYSIS_MS +
        ALERT_HOLD_MS;

      hubAcked = false;

      ackReported = false;

      hubSend(
        HAZARD_EARTHQUAKE,
        alertSeverity
      );

      lastHeartbeat =
        now;
    }
  }

  if (eventActive) {

    if (
      vibration >
      peakVibration
    ) {
      peakVibration =
        vibration;
    }

    if (
      totalAcceleration >
      peakAcceleration
    ) {
      peakAcceleration =
        totalAcceleration;
    }

    if (
      vibration >
      peakPGA
    ) {
      peakPGA =
        vibration;
    }

    if (
      now - eventStart >=
      EVENT_ANALYSIS_MS
    ) {

      float magnitude =
        estimateMagnitude(
          peakPGA
        );

      int intensity =
        estimateIntensity(
          peakPGA
        );

      Serial.println();
      Serial.println(
        "EARTHQUAKE DETECTED"
      );

      Serial.print(
        "TIME: "
      );

      Serial.println(
        (long)eventEpoch
      );

      Serial.print(
        "MAGNITUDE: "
      );

      Serial.println(
        magnitude,
        1
      );

      Serial.print(
        "INTENSITY: "
      );

      Serial.println(
        mmiString(intensity)
      );

      Serial.print(
        "ACCELERATION: "
      );

      Serial.print(
        peakAcceleration,
        3
      );

      Serial.println(
        " g"
      );

      Serial.print(
        "VIBRATION: "
      );

      Serial.print(
        peakVibration,
        3
      );

      Serial.println(
        " g"
      );

      Serial.println();

      buzzerOff();

      alertSeverity =
        severityFromMMI(
          intensity
        );

      alertUntil =
        now +
        ALERT_HOLD_MS;

      hubSend(
        HAZARD_EARTHQUAKE,
        alertSeverity
      );

      lastHeartbeat =
        now;

      eventActive =
        false;

      lastEvent =
        now;

      peakVibration =
        0.0f;

      peakAcceleration =
        0.0f;

      peakPGA =
        0.0f;
    }
  }

  delay(
    SAMPLE_PERIOD_MS
  );
}