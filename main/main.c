#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

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
#define ADXL345_RANGE_2G 0x00
#define ADXL345_RANGE_4G 0x01
#define ADXL345_RANGE_8G 0x02
#define ADXL345_RANGE_16G 0x03
#define ADXL345_RANGE_SELECTED ADXL345_RANGE_16G
#define ADXL345_DATAFMT_VALUE (ADXL345_FULL_RES | ADXL345_RANGE_SELECTED)
#define ADXL345_BWRATE_100HZ 0x0A
#define ADXL345_POWERCTL_MEASURE 0x08
/* FULL_RES keeps 3.9 mg/LSB at every range, so the scale stays valid for 2/4/8/16 g. */
#define ADXL345_G_PER_LSB 0.0039f

#define SAMPLE_RATE_HZ 100
#define SAMPLE_PERIOD_MS 10

#define CALIBRATION_SECONDS 5
#define CALIBRATION_SAMPLES (CALIBRATION_SECONDS * SAMPLE_RATE_HZ)
#define CALIBRATION_MAX_ATTEMPTS 3
#define CALIBRATION_MAX_NOISE_G 0.02f

#define GRAVITY_ALPHA 0.005f

#define STA_SAMPLES 10
#define LTA_SAMPLES 100
#define DETECTOR_HISTORY_SAMPLES (STA_SAMPLES + LTA_SAMPLES)
#define STA_LTA_TRIGGER 3.0f

#define MIN_DYNAMIC_G 0.025f
#define NOISE_MULTIPLIER 6.0f

#define VALIDATION_MS 1200
#define VALIDATION_SAMPLES (VALIDATION_MS / SAMPLE_PERIOD_MS)
#define VALIDATION_MIN_ACTIVE_RATIO 0.35f
#define VALIDATION_MIN_SECOND_ACTIVE 10
#define VALIDATION_MIN_STRONG_HITS 3
#define VALIDATION_STRONG_MULTIPLIER 2.0f
#define VALIDATION_MIN_DIRECTION_CHANGES 4
#define VALIDATION_MAX_QUIET_SAMPLES 40
#define VALIDATION_MIN_PEAK_MULTIPLIER 1.5f

#define EVENT_COOLDOWN_MS 10000

#define WAVEFORM_RATE_HZ 10
#define WAVEFORM_DECIMATION (SAMPLE_RATE_HZ / WAVEFORM_RATE_HZ)
#define WAVEFORM_PRE_SECONDS 2
#define WAVEFORM_POST_SECONDS 6
#define WAVEFORM_PRE_SAMPLES (WAVEFORM_PRE_SECONDS * WAVEFORM_RATE_HZ)
#define WAVEFORM_POST_SAMPLES (WAVEFORM_POST_SECONDS * WAVEFORM_RATE_HZ)
#define WAVEFORM_TOTAL_SAMPLES (WAVEFORM_PRE_SAMPLES + WAVEFORM_POST_SAMPLES)

#define ASSUMED_SOURCE_DISTANCE_KM 10.0f

#define ESPNOW_CHANNEL 1
#define PACKET_VERSION 1

#define MSG_EVENT_START 1
#define MSG_WAVEFORM 2
#define MSG_EVENT_FINAL 3

#define LEVEL_MILD 1
#define LEVEL_MODERATE 2
#define LEVEL_STRONG 3
#define LEVEL_VERY_STRONG 4

#define NODE_ID 4

#define BUZZER_QUEUE_LENGTH 4
#define BUZZER_BEEP_COUNT 3
#define BUZZER_ON_MS 250
#define BUZZER_OFF_MS 180

#define ESPNOW_TX_QUEUE_LENGTH 8
#define ESPNOW_TX_GAP_MS 20
#define ESPNOW_TX_RETRIES 3

#define DEBUG_QUEUE_LENGTH 16
#define DEBUG_REPORT_MS 1000

#define DETECTOR_TASK_PRIORITY 6
#define BUZZER_TASK_PRIORITY 5
#define ESPNOW_TX_TASK_PRIORITY 4
#define DEBUG_TASK_PRIORITY 1

#define DETECTOR_TASK_STACK 6144
#define BUZZER_TASK_STACK 2048
#define ESPNOW_TX_TASK_STACK 3072
#define DEBUG_TASK_STACK 4096

#define BOOT_SETTLE_MS 300
#define ADXL_INIT_ATTEMPTS 10
#define READ_FAIL_RESTART_SAMPLES 200
#define FATAL_RESTART_DELAY_MS 3000

#define SILENCE_SYSTEM_LOGS 1
#define STATUS_BEEPS_ENABLED 1

_Static_assert(SAMPLE_RATE_HZ % WAVEFORM_RATE_HZ == 0, "waveform rate must divide sample rate");
_Static_assert(SAMPLE_RATE_HZ * SAMPLE_PERIOD_MS == 1000, "sample rate and period mismatch");

typedef struct {
    uint8_t version;
    uint8_t type;
    uint8_t node_id;
    uint8_t level;
    uint8_t intensity;
    uint8_t reserved;
    uint16_t sequence;
    float peak_acceleration_g;
    float peak_vibration_g;
    float rms_vibration_g;
    float duration_s;
    float magnitude_est;
} event_packet_t;

typedef struct {
    uint8_t version;
    uint8_t type;
    uint8_t node_id;
    uint8_t reserved1;
    uint8_t sample_count;
    uint8_t reserved2;
    uint16_t sequence;
    int16_t samples[WAVEFORM_TOTAL_SAMPLES];
} waveform_packet_t;

_Static_assert(sizeof(event_packet_t) <= ESP_NOW_MAX_DATA_LEN, "event packet too large");
_Static_assert(sizeof(waveform_packet_t) <= ESP_NOW_MAX_DATA_LEN, "waveform packet too large");

typedef struct {
    uint8_t count;
    uint16_t on_ms;
    uint16_t off_ms;
} buzzer_command_t;

typedef struct {
    uint8_t length;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
} tx_item_t;

typedef enum {
    STATE_NORMAL = 0,
    STATE_CANDIDATE,
    STATE_EVENT
} detector_state_t;

typedef struct {
    int samples;
    int active_count;
    int second_active;
    int strong_hits;
    int quiet_count;
    int max_quiet;
    int direction_changes;
    float peak_vibration;
    float peak_pga;
    float rms_sum_squared;
    uint32_t rms_samples;
    float prev_dx;
    float prev_dy;
    float prev_dz;
    float prev_vibration;
    bool prev_valid;
    int64_t start_us;
    int64_t last_active_us;
} candidate_t;

typedef struct {
    uint16_t sequence;
    int64_t start_us;
    int64_t last_active_us;
    float peak_pga;
    float peak_vibration;
    float rms_sum_squared;
    uint32_t rms_samples;
} event_stats_t;

typedef enum {
    DEBUG_RECORD_CANDIDATE = 1,
    DEBUG_RECORD_VALIDATION,
    DEBUG_RECORD_CONFIRMED,
    DEBUG_RECORD_FINAL
} debug_record_type_t;

typedef struct {
    uint8_t type;
    uint8_t confirmed;
    uint8_t buzzer_queued;
    uint8_t tx_queued;
    uint8_t waveform_queued;
    uint8_t reserved;
    uint16_t sequence;
    int level;
    int intensity;
    int second_active;
    int strong_hits;
    int direction_changes;
    int max_quiet;
    float active_ratio;
    float threshold;
    float peak_vibration;
    float peak_pga;
    float rms_vibration;
    float duration;
    float magnitude;
} debug_record_t;

typedef struct {
    float vibration;
    float pga;
    float threshold;
    float sta_lta;
    uint8_t amplitude_trigger;
    uint8_t onset_trigger;
    uint8_t state;
    uint16_t sequence;
    int candidate_samples;
    int candidate_active;
    int candidate_strong;
    int candidate_direction;
    int waveform_count;
    float peak_pga;
    float peak_vibration;
} debug_snapshot_t;

static const uint8_t master_mac[6] = {
    0x98, 0xF4, 0xAB, 0x3A, 0x62, 0x00
};

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t adxl_device;

static QueueHandle_t buzzer_queue;
static QueueHandle_t espnow_tx_queue;
static QueueHandle_t debug_queue;
static TaskHandle_t debug_task_handle;

static portMUX_TYPE snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static debug_snapshot_t debug_snapshot;

static volatile uint32_t stat_tx_ok;
static volatile uint32_t stat_tx_fail;
static volatile uint32_t stat_tx_dropped;
static volatile uint32_t stat_i2c_failures;

static float gravity_x;
static float gravity_y;
static float gravity_z;
static float noise_rms;

static float detector_history[DETECTOR_HISTORY_SAMPLES];
static int detector_index;
static int detector_count;

static float prebuffer[WAVEFORM_PRE_SAMPLES];
static int prebuffer_index;
static int prebuffer_count;

static uint16_t event_sequence;

static bool buzzer_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << BUZZER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    if (gpio_config(&config) != ESP_OK) {
        return false;
    }

    gpio_set_level(BUZZER_GPIO, 0);
    return true;
}

static void buzzer_on(void)
{
    gpio_set_level(BUZZER_GPIO, 1);
}

static void buzzer_off(void)
{
    gpio_set_level(BUZZER_GPIO, 0);
}

static bool buzzer_enqueue(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
    if (buzzer_queue == NULL) {
        return false;
    }

    buzzer_command_t command = {
        .count = count,
        .on_ms = on_ms,
        .off_ms = off_ms
    };

    return xQueueSend(buzzer_queue, &command, 0) == pdTRUE;
}

static bool buzzer_alert(void)
{
    return buzzer_enqueue(BUZZER_BEEP_COUNT, BUZZER_ON_MS, BUZZER_OFF_MS);
}

static void buzzer_task(void *arg)
{
    (void)arg;
    buzzer_command_t command;

    while (1) {
        if (xQueueReceive(buzzer_queue, &command, portMAX_DELAY) == pdTRUE) {
            for (uint8_t i = 0; i < command.count; i++) {
                buzzer_on();
                vTaskDelay(pdMS_TO_TICKS(command.on_ms));
                buzzer_off();

                if (i + 1 < command.count) {
                    vTaskDelay(pdMS_TO_TICKS(command.off_ms));
                }
            }
        }
    }
}

static void __attribute__((noreturn)) fatal_error(int beeps)
{
    buzzer_init();

#if STATUS_BEEPS_ENABLED
    for (int i = 0; i < beeps; i++) {
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(100));
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
#else
    (void)beeps;
#endif

    vTaskDelay(pdMS_TO_TICKS(FATAL_RESTART_DELAY_MS));
    esp_restart();
}

static esp_err_t adxl_write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(adxl_device, data, sizeof(data), 1000);
}

static esp_err_t adxl_read_register(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(adxl_device, &reg, 1, data, length, 1000);
}

static bool i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    if (i2c_new_master_bus(&bus_config, &i2c_bus) != ESP_OK) {
        return false;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_ADDRESS,
        .scl_speed_hz = 100000
    };

    return i2c_master_bus_add_device(i2c_bus, &device_config, &adxl_device) == ESP_OK;
}

static bool adxl_configure(void)
{
    uint8_t device_id = 0;

    if (adxl_read_register(ADXL345_REG_DEVID, &device_id, 1) != ESP_OK) {
        return false;
    }

    if (device_id != ADXL345_DEVICE_ID) {
        return false;
    }

    if (adxl_write_register(ADXL345_REG_DATAFMT, ADXL345_DATAFMT_VALUE) != ESP_OK) {
        return false;
    }

    if (adxl_write_register(ADXL345_REG_BWRATE, ADXL345_BWRATE_100HZ) != ESP_OK) {
        return false;
    }

    if (adxl_write_register(ADXL345_REG_POWERCTL, ADXL345_POWERCTL_MEASURE) != ESP_OK) {
        return false;
    }

    return true;
}

static bool adxl_init(void)
{
    for (int attempt = 0; attempt < ADXL_INIT_ATTEMPTS; attempt++) {
        if (adxl_configure()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return false;
}

static bool adxl_read_g(float *x, float *y, float *z)
{
    uint8_t data[6];

    if (adxl_read_register(ADXL345_REG_DATAX0, data, sizeof(data)) != ESP_OK) {
        return false;
    }

    int16_t raw_x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    *x = raw_x * ADXL345_G_PER_LSB;
    *y = raw_y * ADXL345_G_PER_LSB;
    *z = raw_z * ADXL345_G_PER_LSB;

    return true;
}

static bool espnow_init(void)
{
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();

    if (esp_wifi_init(&wifi_config) != ESP_OK) {
        return false;
    }

    if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) {
        return false;
    }

    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) {
        return false;
    }

    if (esp_wifi_start() != ESP_OK) {
        return false;
    }

    if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
        return false;
    }

    if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        return false;
    }

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, master_mac, sizeof(master_mac));
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    return esp_now_add_peer(&peer) == ESP_OK;
}

static bool send_packet(const void *data, size_t length)
{
    if (espnow_tx_queue == NULL || length > ESP_NOW_MAX_DATA_LEN) {
        stat_tx_dropped++;
        return false;
    }

    tx_item_t item;
    item.length = (uint8_t)length;
    memcpy(item.data, data, length);

    if (xQueueSend(espnow_tx_queue, &item, 0) != pdTRUE) {
        stat_tx_dropped++;
        return false;
    }

    return true;
}

static void espnow_tx_task(void *arg)
{
    (void)arg;
    tx_item_t item;

    while (1) {
        if (xQueueReceive(espnow_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            bool sent = false;

            for (int attempt = 0; attempt <= ESPNOW_TX_RETRIES && !sent; attempt++) {
                sent = esp_now_send(master_mac, item.data, item.length) == ESP_OK;

                if (!sent) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            if (sent) {
                stat_tx_ok++;
            } else {
                stat_tx_fail++;
            }

            vTaskDelay(pdMS_TO_TICKS(ESPNOW_TX_GAP_MS));
        }
    }
}

static float vector_magnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static float horizontal_magnitude(float x, float y)
{
    return sqrtf(x * x + y * y);
}

static void update_gravity(float ax, float ay, float az)
{
    gravity_x += GRAVITY_ALPHA * (ax - gravity_x);
    gravity_y += GRAVITY_ALPHA * (ay - gravity_y);
    gravity_z += GRAVITY_ALPHA * (az - gravity_z);
}

static void get_dynamic_acceleration(float ax, float ay, float az, float *dx, float *dy, float *dz)
{
    *dx = ax - gravity_x;
    *dy = ay - gravity_y;
    *dz = az - gravity_z;
}

static void detector_reset(void)
{
    memset(detector_history, 0, sizeof(detector_history));
    detector_index = 0;
    detector_count = 0;
}

static float detector_update(float vibration)
{
    detector_history[detector_index] = vibration * vibration;
    detector_index = (detector_index + 1) % DETECTOR_HISTORY_SAMPLES;

    if (detector_count < DETECTOR_HISTORY_SAMPLES) {
        detector_count++;
    }

    if (detector_count < DETECTOR_HISTORY_SAMPLES) {
        return 0.0f;
    }

    int newest = (detector_index + DETECTOR_HISTORY_SAMPLES - 1) % DETECTOR_HISTORY_SAMPLES;
    float sta_sum = 0.0f;
    float lta_sum = 0.0f;

    for (int i = 0; i < DETECTOR_HISTORY_SAMPLES; i++) {
        int index = (newest + DETECTOR_HISTORY_SAMPLES - i) % DETECTOR_HISTORY_SAMPLES;

        if (i < STA_SAMPLES) {
            sta_sum += detector_history[index];
        } else {
            lta_sum += detector_history[index];
        }
    }

    float sta_rms = sqrtf(sta_sum / STA_SAMPLES);
    float lta_rms = sqrtf(lta_sum / LTA_SAMPLES);

    if (lta_rms < 0.000001f) {
        return 0.0f;
    }

    return sta_rms / lta_rms;
}

static void prebuffer_reset(void)
{
    memset(prebuffer, 0, sizeof(prebuffer));
    prebuffer_index = 0;
    prebuffer_count = 0;
}

static void prebuffer_add(float value)
{
    prebuffer[prebuffer_index] = value;
    prebuffer_index = (prebuffer_index + 1) % WAVEFORM_PRE_SAMPLES;

    if (prebuffer_count < WAVEFORM_PRE_SAMPLES) {
        prebuffer_count++;
    }
}

static void prebuffer_copy(float *destination)
{
    for (int i = 0; i < WAVEFORM_PRE_SAMPLES; i++) {
        destination[i] = prebuffer[(prebuffer_index + i) % WAVEFORM_PRE_SAMPLES];
    }
}

static float get_detection_threshold(void)
{
    float threshold = noise_rms * NOISE_MULTIPLIER;

    if (threshold < MIN_DYNAMIC_G) {
        threshold = MIN_DYNAMIC_G;
    }

    return threshold;
}

static int get_level(float pga_g)
{
    if (pga_g >= 0.30f) {
        return LEVEL_VERY_STRONG;
    }
    if (pga_g >= 0.15f) {
        return LEVEL_STRONG;
    }
    if (pga_g >= 0.05f) {
        return LEVEL_MODERATE;
    }
    return LEVEL_MILD;
}

static const char *level_string(int level)
{
    switch (level) {
        case LEVEL_MILD: return "MILD";
        case LEVEL_MODERATE: return "MODERATE";
        case LEVEL_STRONG: return "STRONG";
        case LEVEL_VERY_STRONG: return "VERY STRONG";
        default: return "UNKNOWN";
    }
}

static float estimate_magnitude(float pga_g)
{
    if (pga_g <= 0.000001f) {
        return 0.0f;
    }

    float magnitude =
        (logf(pga_g) +
         logf(ASSUMED_SOURCE_DISTANCE_KM + 7.28f) +
         2.501f) / 0.623f;

    if (magnitude < 0.0f) {
        magnitude = 0.0f;
    }

    if (magnitude > 9.9f) {
        magnitude = 9.9f;
    }

    return magnitude;
}

static int estimate_intensity(float pga_g)
{
    if (pga_g <= 0.0f) {
        return 0;
    }

    float pga_percent_g = pga_g * 100.0f;
    float intensity = 3.66f * log10f(pga_percent_g) + 1.99f;
    int value = (int)lroundf(intensity);

    if (value < 1) {
        value = 1;
    }

    if (value > 8) {
        value = 8;
    }

    return value;
}

static const char *intensity_string(int intensity)
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

static void debug_push(const debug_record_t *record)
{
    if (debug_queue != NULL) {
        xQueueSend(debug_queue, record, 0);
    }
}

static void snapshot_publish(const debug_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&snapshot_lock);
    debug_snapshot = *snapshot;
    portEXIT_CRITICAL(&snapshot_lock);
}

static void snapshot_read(debug_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&snapshot_lock);
    *snapshot = debug_snapshot;
    portEXIT_CRITICAL(&snapshot_lock);
}

static bool calibrate_sensor(void)
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        float ax;
        float ay;
        float az;

        if (!adxl_read_g(&ax, &ay, &az)) {
            return false;
        }

        sum_x += ax;
        sum_y += ay;
        sum_z += az;

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    gravity_x = sum_x / CALIBRATION_SAMPLES;
    gravity_y = sum_y / CALIBRATION_SAMPLES;
    gravity_z = sum_z / CALIBRATION_SAMPLES;

    float noise_sum = 0.0f;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        float ax;
        float ay;
        float az;

        if (!adxl_read_g(&ax, &ay, &az)) {
            return false;
        }

        float motion = vector_magnitude(ax - gravity_x, ay - gravity_y, az - gravity_z);
        noise_sum += motion * motion;

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    noise_rms = sqrtf(noise_sum / CALIBRATION_SAMPLES);

    detector_reset();
    prebuffer_reset();

    return true;
}

static bool send_event_start(uint16_t sequence, float trigger_pga)
{
    event_packet_t packet = {
        .version = PACKET_VERSION,
        .type = MSG_EVENT_START,
        .node_id = NODE_ID,
        .level = LEVEL_MILD,
        .intensity = 0,
        .reserved = 0,
        .sequence = sequence,
        .peak_acceleration_g = trigger_pga,
        .peak_vibration_g = 0.0f,
        .rms_vibration_g = 0.0f,
        .duration_s = 0.0f,
        .magnitude_est = 0.0f
    };

    return send_packet(&packet, sizeof(packet));
}

static bool send_waveform(uint16_t sequence, const float *waveform)
{
    waveform_packet_t packet = {
        .version = PACKET_VERSION,
        .type = MSG_WAVEFORM,
        .node_id = NODE_ID,
        .reserved1 = 0,
        .sample_count = WAVEFORM_TOTAL_SAMPLES,
        .reserved2 = 0,
        .sequence = sequence
    };

    for (int i = 0; i < WAVEFORM_TOTAL_SAMPLES; i++) {
        float scaled = waveform[i] * 1000.0f;

        if (scaled > 32767.0f) {
            scaled = 32767.0f;
        }

        if (scaled < -32768.0f) {
            scaled = -32768.0f;
        }

        packet.samples[i] = (int16_t)lroundf(scaled);
    }

    return send_packet(&packet, sizeof(packet));
}

static bool send_event_final(
    uint16_t sequence,
    float peak_pga,
    float peak_vibration,
    float rms_vibration,
    float duration
)
{
    event_packet_t packet = {
        .version = PACKET_VERSION,
        .type = MSG_EVENT_FINAL,
        .node_id = NODE_ID,
        .level = (uint8_t)get_level(peak_pga),
        .intensity = (uint8_t)estimate_intensity(peak_pga),
        .reserved = 0,
        .sequence = sequence,
        .peak_acceleration_g = peak_pga,
        .peak_vibration_g = peak_vibration,
        .rms_vibration_g = rms_vibration,
        .duration_s = duration,
        .magnitude_est = estimate_magnitude(peak_pga)
    };

    return send_packet(&packet, sizeof(packet));
}

static void debug_print_record(const debug_record_t *record)
{
    switch (record->type) {
        case DEBUG_RECORD_CANDIDATE:
            printf("\nCANDIDATE EVENT\nValidating for %d ms...\n\n", VALIDATION_MS);
            break;

        case DEBUG_RECORD_VALIDATION:
            printf(
                "\nVALIDATION RESULT\n"
                "Active ratio:       %.2f\n"
                "Second-half active: %d\n"
                "Strong hits:        %d\n"
                "Direction changes:  %d\n"
                "Max quiet gap:      %d samples\n"
                "Peak vibration:     %.4f g\n"
                "Peak PGA:           %.4f g\n"
                "Threshold:          %.4f g\n"
                "%s\n\n",
                record->active_ratio,
                record->second_active,
                record->strong_hits,
                record->direction_changes,
                record->max_quiet,
                record->peak_vibration,
                record->peak_pga,
                record->threshold,
                record->confirmed ? "EARTHQUAKE CONFIRMED" : "FALSE TRIGGER - DISCARDED"
            );
            break;

        case DEBUG_RECORD_CONFIRMED:
            printf(
                "################################\n"
                "EARTHQUAKE CONFIRMED\n"
                "SEQUENCE: %u\n"
                "BUZZER: %s\n"
                "EVENT_START: %s\n"
                "################################\n\n",
                record->sequence,
                record->buzzer_queued ? "QUEUED" : "QUEUE FULL",
                record->tx_queued ? "QUEUED" : "DROPPED"
            );
            break;

        case DEBUG_RECORD_FINAL:
            printf(
                "================================\n"
                "EARTHQUAKE DETECTED\n"
                "================================\n"
                "LEVEL:            %s\n"
                "PEAK PGA:         %.4f g\n"
                "PEAK VIBRATION:   %.4f g\n"
                "RMS VIBRATION:    %.4f g\n"
                "SHAKING DURATION: %.2f s\n"
                "MAGNITUDE EST:    %.1f\n"
                "INTENSITY EST:    %s\n"
                "SEQUENCE:         %u\n"
                "WAVEFORM:         %s\n"
                "EVENT FINAL:      %s\n"
                "================================\n\n",
                level_string(record->level),
                record->peak_pga,
                record->peak_vibration,
                record->rms_vibration,
                record->duration,
                record->magnitude,
                intensity_string(record->intensity),
                record->sequence,
                record->waveform_queued ? "QUEUED" : "DROPPED",
                record->tx_queued ? "QUEUED" : "DROPPED"
            );
            break;

        default:
            break;
    }
}

static void debug_print_snapshot(void)
{
    debug_snapshot_t snapshot;
    snapshot_read(&snapshot);

    const char *state = snapshot.state == STATE_EVENT ? "EVENT" :
                        snapshot.state == STATE_CANDIDATE ? "CANDIDATE" : "NORMAL";

    printf(
        "DEBUG | %s | VIB=%.4f g | PGA=%.4f g | TH=%.4f g | STA/LTA=%.2f | AMP=%d | ONSET=%d | TX ok/fail/drop=%lu/%lu/%lu | I2C fails=%lu\n",
        state,
        snapshot.vibration,
        snapshot.pga,
        snapshot.threshold,
        snapshot.sta_lta,
        snapshot.amplitude_trigger,
        snapshot.onset_trigger,
        (unsigned long)stat_tx_ok,
        (unsigned long)stat_tx_fail,
        (unsigned long)stat_tx_dropped,
        (unsigned long)stat_i2c_failures
    );

    if (snapshot.state == STATE_CANDIDATE) {
        printf(
            "        candidate: samples=%d active=%d strong=%d direction=%d peak=%.4f g\n",
            snapshot.candidate_samples,
            snapshot.candidate_active,
            snapshot.candidate_strong,
            snapshot.candidate_direction,
            snapshot.peak_vibration
        );
    } else if (snapshot.state == STATE_EVENT) {
        printf(
            "        event: seq=%u waveform=%d/%d peak_pga=%.4f g peak_vib=%.4f g\n",
            snapshot.sequence,
            snapshot.waveform_count,
            WAVEFORM_TOTAL_SAMPLES,
            snapshot.peak_pga,
            snapshot.peak_vibration
        );
    }
}

static void debug_task(void *arg)
{
    (void)arg;

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    printf("\n================================\n");
    printf("EARTHQUAKE ALERT SLAVE\n");
    printf("================================\n");
    printf("NODE ID:           %d\n", NODE_ID);
    printf("CHANNEL:           %d\n", ESPNOW_CHANNEL);
    printf("MASTER:            %02X:%02X:%02X:%02X:%02X:%02X\n",
           master_mac[0], master_mac[1], master_mac[2],
           master_mac[3], master_mac[4], master_mac[5]);
    printf("SAMPLE RATE:       %d Hz\n", SAMPLE_RATE_HZ);
    printf("ADXL345 RANGE:     +/-%d g full resolution, %.4f g/LSB\n",
           ADXL345_RANGE_SELECTED == ADXL345_RANGE_2G ? 2 :
           ADXL345_RANGE_SELECTED == ADXL345_RANGE_4G ? 4 :
           ADXL345_RANGE_SELECTED == ADXL345_RANGE_8G ? 8 : 16,
           (double)ADXL345_G_PER_LSB);
    printf("VALIDATION:        %d ms\n", VALIDATION_MS);
    printf("WAVEFORM RANGE:    -%d s to +%d s @ %d Hz\n",
           WAVEFORM_PRE_SECONDS, WAVEFORM_POST_SECONDS, WAVEFORM_RATE_HZ);
    printf("================================\n");
    printf("CALIBRATION COMPLETE (%d s gravity + %d s noise)\n", CALIBRATION_SECONDS, CALIBRATION_SECONDS);
    printf("GRAVITY: X=%.4f Y=%.4f Z=%.4f g\n", gravity_x, gravity_y, gravity_z);
    printf("NOISE RMS: %.5f g\n", noise_rms);
    printf("DETECTION THRESHOLD: %.5f g\n", get_detection_threshold());
    printf("READY\n\n");

    debug_record_t record;
    TickType_t last_report = xTaskGetTickCount();

    while (1) {
        if (xQueueReceive(debug_queue, &record, pdMS_TO_TICKS(100)) == pdTRUE) {
            debug_print_record(&record);
        }

        TickType_t now = xTaskGetTickCount();

        if (now - last_report >= pdMS_TO_TICKS(DEBUG_REPORT_MS)) {
            last_report = now;
            debug_print_snapshot();
        }
    }
}

static void detector_task(void *arg)
{
    (void)arg;

    const float threshold = get_detection_threshold();

    bool candidate_active = false;
    bool event_active = false;

    candidate_t candidate;
    event_stats_t event;
    memset(&candidate, 0, sizeof(candidate));
    memset(&event, 0, sizeof(event));

    float waveform[WAVEFORM_TOTAL_SAMPLES];
    int waveform_count = 0;
    int waveform_divider = 0;

    int64_t last_event_us = -EVENT_COOLDOWN_MS * 1000LL;
    uint32_t consecutive_read_failures = 0;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        int64_t now_us = esp_timer_get_time();

        float ax;
        float ay;
        float az;

        if (!adxl_read_g(&ax, &ay, &az)) {
            stat_i2c_failures++;
            consecutive_read_failures++;

            if (consecutive_read_failures >= READ_FAIL_RESTART_SAMPLES) {
                esp_restart();
            }

            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        consecutive_read_failures = 0;

        float dx;
        float dy;
        float dz;

        get_dynamic_acceleration(ax, ay, az, &dx, &dy, &dz);

        float vibration = vector_magnitude(dx, dy, dz);
        float pga = horizontal_magnitude(dx, dy);
        float sta_lta = detector_update(vibration);

        bool amplitude_trigger = vibration >= threshold;
        bool onset_trigger = amplitude_trigger && sta_lta >= STA_LTA_TRIGGER;

        waveform_divider++;
        bool waveform_sample_due = false;

        if (waveform_divider >= WAVEFORM_DECIMATION) {
            waveform_divider = 0;
            waveform_sample_due = true;
        }

        if (!event_active && waveform_sample_due) {
            prebuffer_add(vibration);
        }

        if (event_active) {
            if (pga > event.peak_pga) {
                event.peak_pga = pga;
            }

            if (vibration > event.peak_vibration) {
                event.peak_vibration = vibration;
            }

            if (amplitude_trigger) {
                event.last_active_us = now_us;
            }

            event.rms_sum_squared += vibration * vibration;
            event.rms_samples++;

            if (waveform_sample_due && waveform_count < WAVEFORM_TOTAL_SAMPLES) {
                waveform[waveform_count++] = vibration;
            }

            if (waveform_count >= WAVEFORM_TOTAL_SAMPLES) {
                float rms_vibration = sqrtf(event.rms_sum_squared / (event.rms_samples > 0 ? event.rms_samples : 1));
                float duration = (float)(event.last_active_us - event.start_us) / 1000000.0f;
                int level = get_level(event.peak_pga);
                int intensity = estimate_intensity(event.peak_pga);
                float magnitude = estimate_magnitude(event.peak_pga);

                bool waveform_queued = send_waveform(event.sequence, waveform);
                bool final_queued = send_event_final(event.sequence, event.peak_pga, event.peak_vibration, rms_vibration, duration);

                debug_record_t final_record = {
                    .type = DEBUG_RECORD_FINAL,
                    .waveform_queued = waveform_queued,
                    .tx_queued = final_queued,
                    .sequence = event.sequence,
                    .level = level,
                    .intensity = intensity,
                    .peak_pga = event.peak_pga,
                    .peak_vibration = event.peak_vibration,
                    .rms_vibration = rms_vibration,
                    .duration = duration,
                    .magnitude = magnitude
                };
                debug_push(&final_record);

                event_active = false;
                last_event_us = now_us;
                waveform_count = 0;
                waveform_divider = 0;
                memset(&event, 0, sizeof(event));
                memset(&candidate, 0, sizeof(candidate));
                detector_reset();
                prebuffer_reset();
            }
        } else {
            if (!candidate_active &&
                onset_trigger &&
                now_us - last_event_us >= EVENT_COOLDOWN_MS * 1000LL &&
                prebuffer_count >= WAVEFORM_PRE_SAMPLES) {

                memset(&candidate, 0, sizeof(candidate));
                candidate_active = true;
                candidate.peak_vibration = vibration;
                candidate.peak_pga = pga;
                candidate.start_us = now_us;
                candidate.last_active_us = now_us;

                prebuffer_copy(waveform);
                waveform_count = WAVEFORM_PRE_SAMPLES;
                waveform_divider = 0;
                waveform_sample_due = false;

                debug_record_t record = {
                    .type = DEBUG_RECORD_CANDIDATE,
                    .threshold = threshold
                };
                debug_push(&record);
            }

            if (candidate_active) {
                candidate.samples++;
                candidate.rms_sum_squared += vibration * vibration;
                candidate.rms_samples++;

                if (amplitude_trigger) {
                    candidate.active_count++;
                    candidate.quiet_count = 0;
                    candidate.last_active_us = now_us;
                } else {
                    candidate.quiet_count++;

                    if (candidate.quiet_count > candidate.max_quiet) {
                        candidate.max_quiet = candidate.quiet_count;
                    }
                }

                if (candidate.samples > VALIDATION_SAMPLES / 2 && amplitude_trigger) {
                    candidate.second_active++;
                }

                if (vibration > candidate.peak_vibration) {
                    candidate.peak_vibration = vibration;
                }

                if (pga > candidate.peak_pga) {
                    candidate.peak_pga = pga;
                }

                if (vibration >= threshold * VALIDATION_STRONG_MULTIPLIER) {
                    candidate.strong_hits++;
                }

                if (candidate.prev_valid) {
                    float dot = dx * candidate.prev_dx + dy * candidate.prev_dy + dz * candidate.prev_dz;

                    if (vibration >= threshold &&
                        candidate.prev_vibration >= threshold &&
                        dot < 0.0f) {
                        candidate.direction_changes++;
                    }
                }

                candidate.prev_dx = dx;
                candidate.prev_dy = dy;
                candidate.prev_dz = dz;
                candidate.prev_vibration = vibration;
                candidate.prev_valid = true;

                if (waveform_sample_due && waveform_count < WAVEFORM_TOTAL_SAMPLES) {
                    waveform[waveform_count++] = vibration;
                }

                if (candidate.samples >= VALIDATION_SAMPLES) {
                    float active_ratio = (float)candidate.active_count / VALIDATION_SAMPLES;
                    bool enough_activity = active_ratio >= VALIDATION_MIN_ACTIVE_RATIO;
                    bool activity_continues = candidate.second_active >= VALIDATION_MIN_SECOND_ACTIVE;
                    bool multiple_strong_samples = candidate.strong_hits >= VALIDATION_MIN_STRONG_HITS;
                    bool direction_is_dynamic = candidate.direction_changes >= VALIDATION_MIN_DIRECTION_CHANGES;
                    bool quiet_gap_ok = candidate.max_quiet <= VALIDATION_MAX_QUIET_SAMPLES;
                    bool large_enough = candidate.peak_vibration >= threshold * VALIDATION_MIN_PEAK_MULTIPLIER;

                    bool confirmed =
                        enough_activity &&
                        activity_continues &&
                        multiple_strong_samples &&
                        direction_is_dynamic &&
                        quiet_gap_ok &&
                        large_enough;

                    debug_record_t validation_record = {
                        .type = DEBUG_RECORD_VALIDATION,
                        .confirmed = confirmed,
                        .active_ratio = active_ratio,
                        .second_active = candidate.second_active,
                        .strong_hits = candidate.strong_hits,
                        .direction_changes = candidate.direction_changes,
                        .max_quiet = candidate.max_quiet,
                        .peak_vibration = candidate.peak_vibration,
                        .peak_pga = candidate.peak_pga,
                        .threshold = threshold
                    };
                    debug_push(&validation_record);

                    if (confirmed) {
                        candidate_active = false;
                        event_active = true;

                        event.sequence = ++event_sequence;
                        event.start_us = candidate.start_us;
                        event.last_active_us = candidate.last_active_us;
                        event.peak_pga = candidate.peak_pga;
                        event.peak_vibration = candidate.peak_vibration;
                        event.rms_sum_squared = candidate.rms_sum_squared;
                        event.rms_samples = candidate.rms_samples;

                        bool buzzer_queued = buzzer_alert();
                        bool start_queued = send_event_start(event.sequence, candidate.peak_pga);

                        debug_record_t confirmed_record = {
                            .type = DEBUG_RECORD_CONFIRMED,
                            .sequence = event.sequence,
                            .buzzer_queued = buzzer_queued,
                            .tx_queued = start_queued,
                            .peak_pga = candidate.peak_pga,
                            .peak_vibration = candidate.peak_vibration
                        };
                        debug_push(&confirmed_record);
                    } else {
                        candidate_active = false;
                        waveform_count = 0;
                        waveform_divider = 0;
                        memset(&candidate, 0, sizeof(candidate));
                        detector_reset();
                    }
                }
            }
        }

        debug_snapshot_t snapshot = {
            .vibration = vibration,
            .pga = pga,
            .threshold = threshold,
            .sta_lta = sta_lta,
            .amplitude_trigger = amplitude_trigger,
            .onset_trigger = onset_trigger,
            .state = event_active ? STATE_EVENT : candidate_active ? STATE_CANDIDATE : STATE_NORMAL,
            .sequence = event.sequence,
            .candidate_samples = candidate.samples,
            .candidate_active = candidate.active_count,
            .candidate_strong = candidate.strong_hits,
            .candidate_direction = candidate.direction_changes,
            .waveform_count = waveform_count,
            .peak_pga = event_active ? event.peak_pga : candidate.peak_pga,
            .peak_vibration = event_active ? event.peak_vibration : candidate.peak_vibration
        };
        snapshot_publish(&snapshot);

        update_gravity(ax, ay, az);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void app_main(void)
{
#if SILENCE_SYSTEM_LOGS
    esp_log_level_set("*", ESP_LOG_NONE);
#endif

    vTaskDelay(pdMS_TO_TICKS(BOOT_SETTLE_MS));

    esp_err_t nvs_err = nvs_flash_init();

    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) {
            fatal_error(1);
        }
        nvs_err = nvs_flash_init();
    }

    if (nvs_err != ESP_OK) {
        fatal_error(1);
    }

    if (esp_netif_init() != ESP_OK) {
        fatal_error(1);
    }

    esp_err_t event_err = esp_event_loop_create_default();
    if (event_err != ESP_OK && event_err != ESP_ERR_INVALID_STATE) {
        fatal_error(1);
    }

    if (!i2c_init()) {
        fatal_error(2);
    }

    if (!adxl_init()) {
        fatal_error(3);
    }

    if (!buzzer_init()) {
        fatal_error(4);
    }

    if (!espnow_init()) {
        fatal_error(5);
    }

    buzzer_queue = xQueueCreate(BUZZER_QUEUE_LENGTH, sizeof(buzzer_command_t));
    espnow_tx_queue = xQueueCreate(ESPNOW_TX_QUEUE_LENGTH, sizeof(tx_item_t));
    debug_queue = xQueueCreate(DEBUG_QUEUE_LENGTH, sizeof(debug_record_t));

    if (buzzer_queue == NULL || espnow_tx_queue == NULL || debug_queue == NULL) {
        fatal_error(6);
    }

    if (xTaskCreate(buzzer_task, "buzzer_task", BUZZER_TASK_STACK, NULL, BUZZER_TASK_PRIORITY, NULL) != pdPASS) {
        fatal_error(6);
    }

    if (xTaskCreate(espnow_tx_task, "espnow_tx_task", ESPNOW_TX_TASK_STACK, NULL, ESPNOW_TX_TASK_PRIORITY, NULL) != pdPASS) {
        fatal_error(6);
    }

    if (xTaskCreate(debug_task, "debug_task", DEBUG_TASK_STACK, NULL, DEBUG_TASK_PRIORITY, &debug_task_handle) != pdPASS) {
        fatal_error(6);
    }

#if STATUS_BEEPS_ENABLED
    buzzer_enqueue(1, 500, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
#endif

    for (int attempt = 1; attempt <= CALIBRATION_MAX_ATTEMPTS; attempt++) {
        if (!calibrate_sensor()) {
            fatal_error(7);
        }

        if (noise_rms <= CALIBRATION_MAX_NOISE_G || attempt == CALIBRATION_MAX_ATTEMPTS) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xTaskNotifyGive(debug_task_handle);

#if STATUS_BEEPS_ENABLED
    buzzer_enqueue(2, 80, 80);
    vTaskDelay(pdMS_TO_TICKS(500));
#endif

    if (xTaskCreate(detector_task, "detector_task", DETECTOR_TASK_STACK, NULL, DETECTOR_TASK_PRIORITY, NULL) != pdPASS) {
        fatal_error(6);
    }
}