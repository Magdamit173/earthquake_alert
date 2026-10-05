#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
#define ADXL345_G_PER_LSB 0.0039f

#define SAMPLE_RATE_HZ 100
#define SAMPLE_PERIOD_MS 10

#define CALIBRATION_SECONDS 5
#define CALIBRATION_SAMPLES (CALIBRATION_SECONDS * SAMPLE_RATE_HZ)
#define CALIBRATION_MAX_ATTEMPTS 3
#define CALIBRATION_MAX_NOISE_G 0.02f
#define CALIBRATION_SETTLE_SAMPLES 100

#define LOWPASS_CUTOFF_HZ 15.0f
#define LOWPASS_ALPHA 0.485f

#define DETECTION_NOISE_MULTIPLIER 5.0f
#define MIN_DYNAMIC_G 0.03f
#define START_CONSECUTIVE_SAMPLES 3
#define VALIDATION_MS 600
#define VALIDATION_SAMPLES (VALIDATION_MS / SAMPLE_PERIOD_MS)
#define VALIDATION_MIN_ACTIVE_RATIO 0.35f
#define VALIDATION_MIN_STRONG_HITS 2
#define VALIDATION_STRONG_MULTIPLIER 2.0f
#define VALIDATION_MIN_PEAK_MULTIPLIER 1.5f
#define EVENT_COOLDOWN_MS 10000

#define WAVEFORM_RATE_HZ 10
#define WAVEFORM_DECIMATION (SAMPLE_RATE_HZ / WAVEFORM_RATE_HZ)
#define CAPTURE_PRE_SECONDS 2
#define CAPTURE_POST_SECONDS 3
#define CAPTURE_SPAN_SECONDS (CAPTURE_PRE_SECONDS + CAPTURE_POST_SECONDS)
#define CAPTURE_PRE_SAMPLES (CAPTURE_PRE_SECONDS * WAVEFORM_RATE_HZ)
#define CAPTURE_POST_SAMPLES (CAPTURE_POST_SECONDS * WAVEFORM_RATE_HZ)
#define CAPTURE_TOTAL_SAMPLES (CAPTURE_PRE_SAMPLES + CAPTURE_POST_SAMPLES)

#define JSON_WAVEFORM_SCALE_G 0.01f
#define JSON_WAVEFORM_SCALE_MG 10

#define ASSUMED_SOURCE_DISTANCE_KM 10.0f

#define ESPNOW_CHANNEL 1
#define PACKET_VERSION 1
#define ESPNOW_PAYLOAD_MAX ESP_NOW_MAX_DATA_LEN_V2

#define NODE_ID 4

#define BUZZER_QUEUE_LENGTH 4
#define BUZZER_BEEP_COUNT 3
#define BUZZER_ON_MS 250
#define BUZZER_OFF_MS 180

#define ESPNOW_TX_QUEUE_LENGTH 4
#define ESPNOW_TX_GAP_MS 20
#define ESPNOW_TX_RETRIES 3
#define EVENT_QUEUE_LENGTH 2

#define DEBUG_QUEUE_LENGTH 16
#define DEBUG_REPORT_MS 1000

#define DETECTOR_TASK_PRIORITY 6
#define BUZZER_TASK_PRIORITY 5
#define ESPNOW_TX_TASK_PRIORITY 4
#define ANALYSIS_TASK_PRIORITY 3
#define DEBUG_TASK_PRIORITY 1

#define DETECTOR_TASK_STACK 6144
#define BUZZER_TASK_STACK 2048
#define ESPNOW_TX_TASK_STACK 3072
#define ANALYSIS_TASK_STACK 6144
#define DEBUG_TASK_STACK 4096

#define BOOT_SETTLE_MS 300
#define ADXL_INIT_ATTEMPTS 10
#define READ_FAIL_RESTART_SAMPLES 200
#define FATAL_RESTART_DELAY_MS 3000

#define SILENCE_SYSTEM_LOGS 1
#define STATUS_BEEPS_ENABLED 1

_Static_assert(SAMPLE_RATE_HZ % WAVEFORM_RATE_HZ == 0, "waveform rate must divide sample rate");
_Static_assert(SAMPLE_RATE_HZ * SAMPLE_PERIOD_MS == 1000, "sample rate and period mismatch");
_Static_assert(CAPTURE_TOTAL_SAMPLES <= 255, "capture too large");

typedef enum {
    STATE_NORMAL = 0,
    STATE_CANDIDATE,
    STATE_CAPTURE
} detector_state_t;

typedef struct {
    uint8_t count;
    uint16_t on_ms;
    uint16_t off_ms;
} buzzer_command_t;

typedef struct {
    uint16_t length;
    uint8_t data[ESPNOW_PAYLOAD_MAX];
} tx_item_t;

typedef struct {
    int samples;
    int active_count;
    int strong_hits;
    float peak_vibration;
    float peak_pga_h;
    int64_t start_us;
    int64_t last_active_us;
    float vx;
    float vy;
    float vz;
    float peak_pgv_h;
    float peak_pgv_v;
    float cav_g_s;
} candidate_t;

typedef struct {
    uint16_t sequence;
    int64_t onset_us;
    int64_t last_active_us;
    float peak_accel;
    float peak_pga_h;
    float peak_pga_v;
    float peak_vibration;
    float peak_x;
    float peak_y;
    float peak_z;
    float peak_pgv_h;
    float peak_pgv_v;
    float vx;
    float vy;
    float vz;
    float cav_g_s;
} event_stats_t;

typedef struct {
    uint16_t sequence;
    int64_t capture_us;
    float h_g[CAPTURE_TOTAL_SAMPLES];
    float v_g[CAPTURE_TOTAL_SAMPLES];
    float peak_accel_g;
    float peak_pga_h_g;
    float peak_pga_v_g;
    float peak_vibration_g;
    float peak_x_g;
    float peak_y_g;
    float peak_z_g;
    float peak_pgv_h_cm_s;
    float peak_pgv_v_cm_s;
    float cav_g_s;
    float duration_s;
    float rms_vibration_g;
    int vertical_axis;
    float threshold_g;
} event_capture_t;

typedef enum {
    DEBUG_RECORD_CANDIDATE = 1,
    DEBUG_RECORD_VALIDATION,
    DEBUG_RECORD_FINAL
} debug_record_type_t;

typedef struct {
    uint8_t type;
    uint8_t confirmed;
    uint8_t buzzer_queued;
    uint8_t tx_queued;
    uint16_t sequence;
    int level;
    int intensity;
    int strong_hits;
    float active_ratio;
    float threshold;
    float peak_accel;
    float peak_pga_h;
    float peak_pga_v;
    float pgv_h;
    float pgv_v;
    float rms_vibration;
    float cav_g_s;
    float duration;
    float dominant_hz;
    float magnitude;
} debug_record_t;

typedef struct {
    float vibration;
    float pga_h;
    float pga_v;
    float threshold;
    uint8_t amplitude_trigger;
    uint8_t onset_trigger;
    uint8_t state;
    uint16_t sequence;
    int candidate_samples;
    int candidate_active;
    int candidate_strong;
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
static QueueHandle_t event_queue;
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
static int vertical_axis;

static float lp_x;
static float lp_y;
static float lp_z;
static bool filter_ready;

static float pre_h[CAPTURE_PRE_SAMPLES];
static float pre_v[CAPTURE_PRE_SAMPLES];
static int prebuffer_index;
static int prebuffer_count;

static uint16_t event_sequence;
static uint32_t espnow_version;

static void filter_reset(void)
{
    lp_x = 0.0f;
    lp_y = 0.0f;
    lp_z = 0.0f;
    filter_ready = false;
}

static void filter_update(float dx, float dy, float dz, float *fx, float *fy, float *fz)
{
    if (!filter_ready) {
        lp_x = dx;
        lp_y = dy;
        lp_z = dz;
        filter_ready = true;
    } else {
        lp_x += LOWPASS_ALPHA * (dx - lp_x);
        lp_y += LOWPASS_ALPHA * (dy - lp_y);
        lp_z += LOWPASS_ALPHA * (dz - lp_z);
    }

    *fx = lp_x;
    *fy = lp_y;
    *fz = lp_z;
}

static float vector_magnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static float horizontal_magnitude(float x, float y, float z)
{
    if (vertical_axis == 0) {
        return sqrtf(y * y + z * z);
    }
    if (vertical_axis == 1) {
        return sqrtf(x * x + z * z);
    }
    return sqrtf(x * x + y * y);
}

static float vertical_value(float x, float y, float z)
{
    if (vertical_axis == 0) {
        return fabsf(x);
    }
    if (vertical_axis == 1) {
        return fabsf(y);
    }
    return fabsf(z);
}

static void velocity_update(float ax, float ay, float az, float *vx, float *vy, float *vz)
{
    const float leak = 0.995f;
    const float dt_scale = 980.665f * (SAMPLE_PERIOD_MS / 1000.0f);

    *vx = leak * (*vx) + ax * dt_scale;
    *vy = leak * (*vy) + ay * dt_scale;
    *vz = leak * (*vz) + az * dt_scale;

    if (fabsf(*vx) < 0.02f) {
        *vx = 0.0f;
    }
    if (fabsf(*vy) < 0.02f) {
        *vy = 0.0f;
    }
    if (fabsf(*vz) < 0.02f) {
        *vz = 0.0f;
    }
}

static float velocity_horizontal(float vx, float vy, float vz)
{
    return horizontal_magnitude(vx, vy, vz);
}

static float velocity_vertical(float vx, float vy, float vz)
{
    if (vertical_axis == 0) {
        return fabsf(vx);
    }
    if (vertical_axis == 1) {
        return fabsf(vy);
    }
    return fabsf(vz);
}

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

    if (esp_now_get_version(&espnow_version) != ESP_OK || espnow_version < 2) {
        return false;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, master_mac, sizeof(master_mac));
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t peer_err = esp_now_add_peer(&peer);
    return peer_err == ESP_OK || peer_err == ESP_ERR_ESPNOW_EXIST;
}

static bool send_packet(const void *data, size_t length)
{
    if (espnow_tx_queue == NULL || length == 0 || length > ESPNOW_PAYLOAD_MAX) {
        stat_tx_dropped++;
        return false;
    }

    tx_item_t item;
    item.length = (uint16_t)length;
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

static void prebuffer_reset(void)
{
    memset(pre_h, 0, sizeof(pre_h));
    memset(pre_v, 0, sizeof(pre_v));
    prebuffer_index = 0;
    prebuffer_count = 0;
}

static void prebuffer_add(float h, float v)
{
    pre_h[prebuffer_index] = h;
    pre_v[prebuffer_index] = v;
    prebuffer_index = (prebuffer_index + 1) % CAPTURE_PRE_SAMPLES;

    if (prebuffer_count < CAPTURE_PRE_SAMPLES) {
        prebuffer_count++;
    }
}

static void prebuffer_copy(float *h, float *v)
{
    int missing = CAPTURE_PRE_SAMPLES - prebuffer_count;

    for (int i = 0; i < missing; i++) {
        h[i] = 0.0f;
        v[i] = 0.0f;
    }

    for (int i = missing; i < CAPTURE_PRE_SAMPLES; i++) {
        int src = (prebuffer_index + i - missing) % CAPTURE_PRE_SAMPLES;
        h[i] = pre_h[src];
        v[i] = pre_v[src];
    }
}

static float get_detection_threshold(void)
{
    float threshold = noise_rms * DETECTION_NOISE_MULTIPLIER;
    if (threshold < MIN_DYNAMIC_G) {
        threshold = MIN_DYNAMIC_G;
    }
    return threshold;
}

static int get_level(float pga_g)
{
    if (pga_g >= 0.30f) {
        return 4;
    }
    if (pga_g >= 0.15f) {
        return 3;
    }
    if (pga_g >= 0.05f) {
        return 2;
    }
    return 1;
}

static const char *level_string(int level)
{
    switch (level) {
        case 1: return "MILD";
        case 2: return "MODERATE";
        case 3: return "STRONG";
        case 4: return "VERY STRONG";
        default: return "UNKNOWN";
    }
}

static int estimate_intensity(float pga_h_g, float pgv_h_cm_s)
{
    if (pga_h_g <= 0.0f) {
        return 1;
    }

    float pga_cm_s2 = pga_h_g * 980.665f;
    float log_pga = log10f(pga_cm_s2 > 0.001f ? pga_cm_s2 : 0.001f);
    float mmi_pga = log_pga <= 1.57f
        ? 1.78f + 1.55f * log_pga
        : -1.60f + 3.70f * log_pga;

    float mmi = mmi_pga;

    if (pgv_h_cm_s > 0.001f) {
        float log_pgv = log10f(pgv_h_cm_s);
        float mmi_pgv = log_pgv <= 0.53f
            ? 3.78f + 1.47f * log_pgv
            : 2.89f + 3.16f * log_pgv;

        if (mmi_pga < 5.0f) {
            mmi = mmi_pga;
        } else if (mmi_pga >= 7.0f) {
            mmi = mmi_pgv;
        } else {
            float w = (mmi_pga - 5.0f) / 2.0f;
            mmi = mmi_pga * (1.0f - w) + mmi_pgv * w;
        }
    }

    int value = (int)lroundf(mmi);
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

static float estimate_magnitude(float pga_h_g, float distance_km)
{
    if (pga_h_g <= 0.000001f || distance_km < 0.0f) {
        return 0.0f;
    }

    float magnitude =
        (logf(pga_h_g) +
         logf(distance_km + 7.28f) +
         2.501f) / 0.623f;

    if (magnitude < 0.0f) {
        magnitude = 0.0f;
    }
    if (magnitude > 9.9f) {
        magnitude = 9.9f;
    }

    return magnitude;
}

static float dominant_frequency_hz(const event_capture_t *capture)
{
    float best_frequency = 0.0f;
    float best_power = 0.0f;

    for (int k = 1; k <= 10; k++) {
        float frequency = 0.5f * (float)k;
        float power = 0.0f;

        for (int axis = 0; axis < 2; axis++) {
            float real = 0.0f;
            float imag = 0.0f;
            const float *samples = axis == 0 ? capture->h_g : capture->v_g;
            float mean = 0.0f;

            for (int n = 0; n < CAPTURE_TOTAL_SAMPLES; n++) {
                mean += samples[n];
            }
            mean /= CAPTURE_TOTAL_SAMPLES;

            for (int n = 0; n < CAPTURE_TOTAL_SAMPLES; n++) {
                float phase = 2.0f * (float)M_PI * frequency * (float)n / (float)WAVEFORM_RATE_HZ;
                float window = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)n / (float)(CAPTURE_TOTAL_SAMPLES - 1));
                float sample = samples[n] - mean;
                real += sample * window * cosf(phase);
                imag -= sample * window * sinf(phase);
            }

            power += real * real + imag * imag;
        }

        if (power > best_power) {
            best_power = power;
            best_frequency = frequency;
        }
    }

    return best_frequency;
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

    float gravity_magnitude = vector_magnitude(gravity_x, gravity_y, gravity_z);
    float abs_x = fabsf(gravity_x);
    float abs_y = fabsf(gravity_y);
    float abs_z = fabsf(gravity_z);

    if (abs_x >= abs_y && abs_x >= abs_z) {
        vertical_axis = 0;
    } else if (abs_y >= abs_z) {
        vertical_axis = 1;
    } else {
        vertical_axis = 2;
    }

    filter_reset();

    float noise_sum = 0.0f;
    int noise_count = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        float ax;
        float ay;
        float az;
        float dx;
        float dy;
        float dz;
        float fx;
        float fy;
        float fz;

        if (!adxl_read_g(&ax, &ay, &az)) {
            return false;
        }

        dx = ax - gravity_x;
        dy = ay - gravity_y;
        dz = az - gravity_z;
        filter_update(dx, dy, dz, &fx, &fy, &fz);

        if (i >= CALIBRATION_SETTLE_SAMPLES) {
            float motion = vector_magnitude(fx, fy, fz);
            noise_sum += motion * motion;
            noise_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    noise_rms = sqrtf(noise_sum / (noise_count > 0 ? noise_count : 1));
    filter_reset();
    prebuffer_reset();

    return gravity_magnitude >= 0.8f && gravity_magnitude <= 1.2f;
}

static bool json_append(char *buffer, size_t capacity, size_t *length, const char *format, ...)
{
    if (*length >= capacity) {
        return false;
    }

    va_list args;
    va_start(args, format);
    int written = vsnprintf(buffer + *length, capacity - *length, format, args);
    va_end(args);

    if (written < 0 || (size_t)written >= capacity - *length) {
        return false;
    }

    *length += (size_t)written;
    return true;
}

static bool build_json(const event_capture_t *capture, char *json, size_t capacity, size_t *length)
{
    *length = 0;

    if (!json_append(json, capacity, length,
        "{\"version\":%d,\"event\":\"earthquake\",\"node\":%d,\"sequence\":%u,"
        "\"uptime_ms\":%llu,\"sample_rate_hz\":%d,\"span_s\":%d,\"pre_s\":%d,\"post_s\":%d,"
        "\"vertical_axis\":%d,\"level\":\"%s\",\"intensity\":%d,\"intensity_roman\":\"%s\",\"intensity_scale\":\"MMI_EST\","
        "\"pga_g\":%.4f,\"pga_h_g\":%.4f,\"pga_v_g\":%.4f,\"pga_h_cm_s2\":%.1f,\"pga_v_cm_s2\":%.1f,\"peak_vibration_g\":%.4f,"
        "\"peak_x_g\":%.4f,\"peak_y_g\":%.4f,\"peak_z_g\":%.4f,\"pgv_h_cm_s\":%.3f,\"pgv_v_cm_s\":%.3f,"
        "\"rms_vibration_g\":%.4f,\"cav_g_s\":%.4f,\"duration_s\":%.3f,\"dominant_hz\":%.1f,"
        "\"magnitude_est\":%.2f,\"magnitude_distance_assumed_km\":%.1f,\"magnitude_note\":\"rough_estimate\","
        "\"calibration_noise_g\":%.5f,\"detection_threshold_g\":%.5f,\"waveform_resolution_g\":%.2f,\"capture_reference\":\"confirmation\",\"h\":[",
        PACKET_VERSION,
        NODE_ID,
        capture->sequence,
        (unsigned long long)(capture->capture_us / 1000LL),
        WAVEFORM_RATE_HZ,
        CAPTURE_SPAN_SECONDS,
        CAPTURE_PRE_SECONDS,
        CAPTURE_POST_SECONDS,
        capture->vertical_axis,
        level_string(get_level(capture->peak_pga_h_g)),
        estimate_intensity(capture->peak_pga_h_g, capture->peak_pgv_h_cm_s),
        intensity_string(estimate_intensity(capture->peak_pga_h_g, capture->peak_pgv_h_cm_s)),
        capture->peak_accel_g,
        capture->peak_pga_h_g,
        capture->peak_pga_v_g,
        capture->peak_pga_h_g * 980.665f,
        capture->peak_pga_v_g * 980.665f,
        capture->peak_vibration_g,
        capture->peak_x_g,
        capture->peak_y_g,
        capture->peak_z_g,
        capture->peak_pgv_h_cm_s,
        capture->peak_pgv_v_cm_s,
        capture->rms_vibration_g,
        capture->cav_g_s,
        capture->duration_s,
        dominant_frequency_hz(capture),
        estimate_magnitude(capture->peak_pga_h_g, ASSUMED_SOURCE_DISTANCE_KM),
        ASSUMED_SOURCE_DISTANCE_KM,
        noise_rms,
        capture->threshold_g,
        JSON_WAVEFORM_SCALE_G)) {
        return false;
    }

    for (int i = 0; i < CAPTURE_TOTAL_SAMPLES; i++) {
        if (!json_append(json, capacity, length, "%s%d", i == 0 ? "" : ",",
                         (int)lroundf(capture->h_g[i] * JSON_WAVEFORM_SCALE_MG))) {
            return false;
        }
    }

    if (!json_append(json, capacity, length, "],\"v\":[")) {
        return false;
    }

    for (int i = 0; i < CAPTURE_TOTAL_SAMPLES; i++) {
        if (!json_append(json, capacity, length, "%s%d", i == 0 ? "" : ",",
                         (int)lroundf(capture->v_g[i] * JSON_WAVEFORM_SCALE_MG))) {
            return false;
        }
    }

    if (!json_append(json, capacity, length, "]}")) {
        return false;
    }

    return *length <= ESPNOW_PAYLOAD_MAX;
}

static void analysis_task(void *arg)
{
    (void)arg;
    event_capture_t capture;
    char json[ESPNOW_PAYLOAD_MAX + 1];

    while (1) {
        if (xQueueReceive(event_queue, &capture, portMAX_DELAY) == pdTRUE) {
            size_t json_length = 0;
            bool json_ok = build_json(&capture, json, sizeof(json), &json_length);
            bool tx_queued = json_ok && send_packet(json, json_length);

            debug_record_t record = {
                .type = DEBUG_RECORD_FINAL,
                .tx_queued = tx_queued,
                .sequence = capture.sequence,
                .level = get_level(capture.peak_pga_h_g),
                .intensity = estimate_intensity(capture.peak_pga_h_g, capture.peak_pgv_h_cm_s),
                .peak_accel = capture.peak_accel_g,
                .peak_pga_h = capture.peak_pga_h_g,
                .peak_pga_v = capture.peak_pga_v_g,
                .pgv_h = capture.peak_pgv_h_cm_s,
                .pgv_v = capture.peak_pgv_v_cm_s,
                .rms_vibration = capture.rms_vibration_g,
                .cav_g_s = capture.cav_g_s,
                .duration = capture.duration_s,
                .dominant_hz = dominant_frequency_hz(&capture),
                .magnitude = estimate_magnitude(capture.peak_pga_h_g, ASSUMED_SOURCE_DISTANCE_KM)
            };

            debug_push(&record);
        }
    }
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
                "Strong hits:        %d\n"
                "Peak acceleration:  %.4f g\n"
                "Peak H PGA:         %.4f g\n"
                "Peak V PGA:         %.4f g\n"
                "Threshold:          %.4f g\n"
                "%s\n\n",
                record->active_ratio,
                record->strong_hits,
                record->peak_accel,
                record->peak_pga_h,
                record->peak_pga_v,
                record->threshold,
                record->confirmed ? "EARTHQUAKE CONFIRMED" : "FALSE TRIGGER - DISCARDED"
            );
            break;

        case DEBUG_RECORD_FINAL:
            printf(
                "================================\n"
                "EARTHQUAKE DETECTED\n"
                "================================\n"
                "NODE ID:           %d\n"
                "SEQUENCE:          %u\n"
                "LEVEL:             %s\n"
                "INTENSITY:         %s\n"
                "PEAK ACCEL:        %.4f g\n"
                "PEAK H PGA:        %.4f g\n"
                "PEAK V PGA:        %.4f g\n"
                "PEAK H PGV:        %.3f cm/s\n"
                "PEAK V PGV:        %.3f cm/s\n"
                "RMS VIBRATION:     %.4f g\n"
                "CAV:               %.4f g*s\n"
                "DURATION:           %.3f s\n"
                "DOMINANT FREQ:     %.1f Hz\n"
                "MAGNITUDE EST:     %.2f\n"
                "DISTANCE ASSUMED:  %.1f km\n"
                "JSON:              %s\n"
                "================================\n\n",
                NODE_ID,
                record->sequence,
                level_string(record->level),
                intensity_string(record->intensity),
                record->peak_accel,
                record->peak_pga_h,
                record->peak_pga_v,
                record->pgv_h,
                record->pgv_v,
                record->rms_vibration,
                record->cav_g_s,
                record->duration,
                record->dominant_hz,
                record->magnitude,
                ASSUMED_SOURCE_DISTANCE_KM,
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

    const char *state = snapshot.state == STATE_CAPTURE ? "CAPTURE" :
                        snapshot.state == STATE_CANDIDATE ? "CANDIDATE" : "NORMAL";

    printf(
        "DEBUG | %s | VIB=%.4f g | PGA H/V=%.4f/%.4f g | TH=%.4f g | AMP=%d | ONSET=%d | TX ok/fail/drop=%lu/%lu/%lu | I2C fails=%lu\n",
        state,
        snapshot.vibration,
        snapshot.pga_h,
        snapshot.pga_v,
        snapshot.threshold,
        snapshot.amplitude_trigger,
        snapshot.onset_trigger,
        (unsigned long)stat_tx_ok,
        (unsigned long)stat_tx_fail,
        (unsigned long)stat_tx_dropped,
        (unsigned long)stat_i2c_failures
    );

    if (snapshot.state == STATE_CANDIDATE) {
        printf(
            "        candidate: samples=%d active=%d strong=%d peak=%.4f g\n",
            snapshot.candidate_samples,
            snapshot.candidate_active,
            snapshot.candidate_strong,
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
    printf("ESP-NOW VERSION:   v%lu\n", (unsigned long)espnow_version);
    printf("MASTER:            %02X:%02X:%02X:%02X:%02X:%02X\n",
           master_mac[0], master_mac[1], master_mac[2],
           master_mac[3], master_mac[4], master_mac[5]);
    printf("SAMPLE RATE:       %d Hz\n", SAMPLE_RATE_HZ);
    printf("CAPTURE RATE:      %d Hz\n", WAVEFORM_RATE_HZ);
    printf("CAPTURE SPAN:      %d s (%d s pre + %d s post)\n",
           CAPTURE_SPAN_SECONDS, CAPTURE_PRE_SECONDS, CAPTURE_POST_SECONDS);
    printf("LOW-PASS:          %.1f Hz\n", LOWPASS_CUTOFF_HZ);
    printf("================================\n");
    printf("CALIBRATION COMPLETE\n");
    printf("GRAVITY:           X=%.4f Y=%.4f Z=%.4f g\n", gravity_x, gravity_y, gravity_z);
    printf("VERTICAL AXIS:     %d\n", vertical_axis);
    printf("NOISE RMS:         %.5f g\n", noise_rms);
    printf("DETECTION THRESH:  %.5f g\n", get_detection_threshold());
    printf("MAGNITUDE DISTANCE: %.1f km assumed\n", ASSUMED_SOURCE_DISTANCE_KM);
    printf("INTENSITY:         instrumental MMI-like estimate\n");
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
    const float sustain_threshold = threshold * 0.5f;

    bool candidate_active = false;
    bool event_active = false;

    candidate_t candidate;
    event_stats_t event;
    event_capture_t capture;
    memset(&candidate, 0, sizeof(candidate));
    memset(&event, 0, sizeof(event));
    memset(&capture, 0, sizeof(capture));

    int capture_count = 0;
    int waveform_divider = 0;
    int onset_streak = 0;
    int64_t last_event_us = -EVENT_COOLDOWN_MS * 1000LL;
    uint32_t consecutive_read_failures = 0;
    TickType_t last_wake = xTaskGetTickCount();

    filter_reset();

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

        float dx = ax - gravity_x;
        float dy = ay - gravity_y;
        float dz = az - gravity_z;
        float fx;
        float fy;
        float fz;
        filter_update(dx, dy, dz, &fx, &fy, &fz);

        float vibration = vector_magnitude(fx, fy, fz);
        float pga_h = horizontal_magnitude(fx, fy, fz);
        float pga_v = vertical_value(fx, fy, fz);
        bool amplitude_trigger = vibration >= threshold;

        if (amplitude_trigger) {
            if (onset_streak < START_CONSECUTIVE_SAMPLES) {
                onset_streak++;
            }
        } else {
            onset_streak = 0;
        }

        bool onset_trigger = onset_streak >= START_CONSECUTIVE_SAMPLES;

        waveform_divider++;
        bool waveform_sample_due = false;
        if (waveform_divider >= WAVEFORM_DECIMATION) {
            waveform_divider = 0;
            waveform_sample_due = true;
        }

        if (!event_active && waveform_sample_due) {
            prebuffer_add(pga_h, pga_v);
        }

        if (event_active) {
            event.cav_g_s += vibration * (SAMPLE_PERIOD_MS / 1000.0f);
            velocity_update(fx, fy, fz, &event.vx, &event.vy, &event.vz);

            float pgv_h = velocity_horizontal(event.vx, event.vy, event.vz);
            float pgv_v = velocity_vertical(event.vx, event.vy, event.vz);

            if (vibration > event.peak_accel) {
                event.peak_accel = vibration;
            }
            if (pga_h > event.peak_pga_h) {
                event.peak_pga_h = pga_h;
            }
            if (pga_v > event.peak_pga_v) {
                event.peak_pga_v = pga_v;
            }
            if (vibration > event.peak_vibration) {
                event.peak_vibration = vibration;
            }
            if (fabsf(fx) > event.peak_x) {
                event.peak_x = fabsf(fx);
            }
            if (fabsf(fy) > event.peak_y) {
                event.peak_y = fabsf(fy);
            }
            if (fabsf(fz) > event.peak_z) {
                event.peak_z = fabsf(fz);
            }
            if (pgv_h > event.peak_pgv_h) {
                event.peak_pgv_h = pgv_h;
            }
            if (pgv_v > event.peak_pgv_v) {
                event.peak_pgv_v = pgv_v;
            }
            if (amplitude_trigger) {
                event.last_active_us = now_us;
            }

            if (waveform_sample_due && capture_count < CAPTURE_TOTAL_SAMPLES) {
                capture.h_g[capture_count] = pga_h;
                capture.v_g[capture_count] = pga_v;
                capture_count++;
            }

            if (capture_count >= CAPTURE_TOTAL_SAMPLES) {
                capture.sequence = event.sequence;
                capture.capture_us = now_us;
                capture.peak_accel_g = event.peak_accel;
                capture.peak_pga_h_g = event.peak_pga_h;
                capture.peak_pga_v_g = event.peak_pga_v;
                capture.peak_vibration_g = event.peak_vibration;
                capture.peak_x_g = event.peak_x;
                capture.peak_y_g = event.peak_y;
                capture.peak_z_g = event.peak_z;
                capture.peak_pgv_h_cm_s = event.peak_pgv_h;
                capture.peak_pgv_v_cm_s = event.peak_pgv_v;
                capture.cav_g_s = event.cav_g_s;
                capture.duration_s = (float)(event.last_active_us - event.onset_us) / 1000000.0f;
                capture.vertical_axis = vertical_axis;
                capture.threshold_g = threshold;

                float sum_sq = 0.0f;
                for (int i = 0; i < CAPTURE_TOTAL_SAMPLES; i++) {
                    sum_sq += capture.h_g[i] * capture.h_g[i] + capture.v_g[i] * capture.v_g[i];
                }
                capture.rms_vibration_g = sqrtf(sum_sq / (float)CAPTURE_TOTAL_SAMPLES);

                bool queued = xQueueSend(event_queue, &capture, 0) == pdTRUE;

                event_active = false;
                candidate_active = false;
                last_event_us = now_us;
                capture_count = 0;
                waveform_divider = 0;
                onset_streak = 0;
                memset(&event, 0, sizeof(event));
                memset(&candidate, 0, sizeof(candidate));
                prebuffer_reset();

                if (!queued) {
                    stat_tx_dropped++;
                }
            }
        } else {
            if (!candidate_active &&
                onset_trigger &&
                now_us - last_event_us >= EVENT_COOLDOWN_MS * 1000LL &&
                prebuffer_count >= CAPTURE_PRE_SAMPLES) {

                memset(&candidate, 0, sizeof(candidate));
                candidate_active = true;
                candidate.peak_vibration = vibration;
                candidate.peak_pga_h = pga_h;
                candidate.start_us = now_us;
                candidate.last_active_us = now_us;
                waveform_divider = 0;
                onset_streak = 0;

                debug_record_t record = {
                    .type = DEBUG_RECORD_CANDIDATE,
                    .threshold = threshold
                };
                debug_push(&record);
            }

            if (candidate_active) {
                candidate.samples++;
                candidate.cav_g_s += vibration * (SAMPLE_PERIOD_MS / 1000.0f);
                velocity_update(fx, fy, fz, &candidate.vx, &candidate.vy, &candidate.vz);

                float pgv_h = velocity_horizontal(candidate.vx, candidate.vy, candidate.vz);
                float pgv_v = velocity_vertical(candidate.vx, candidate.vy, candidate.vz);

                if (vibration >= sustain_threshold) {
                    candidate.active_count++;
                }
                if (vibration >= threshold * VALIDATION_STRONG_MULTIPLIER) {
                    candidate.strong_hits++;
                }
                if (vibration > candidate.peak_vibration) {
                    candidate.peak_vibration = vibration;
                }
                if (pga_h > candidate.peak_pga_h) {
                    candidate.peak_pga_h = pga_h;
                }
                if (vibration >= threshold) {
                    candidate.last_active_us = now_us;
                }
                if (pgv_h > candidate.peak_pgv_h) {
                    candidate.peak_pgv_h = pgv_h;
                }
                if (pgv_v > candidate.peak_pgv_v) {
                    candidate.peak_pgv_v = pgv_v;
                }

                if (candidate.samples >= VALIDATION_SAMPLES) {
                    float active_ratio = (float)candidate.active_count / (float)VALIDATION_SAMPLES;
                    bool enough_activity = active_ratio >= VALIDATION_MIN_ACTIVE_RATIO;
                    bool enough_strong_hits = candidate.strong_hits >= VALIDATION_MIN_STRONG_HITS;
                    bool large_enough = candidate.peak_vibration >= threshold * VALIDATION_MIN_PEAK_MULTIPLIER;
                    bool confirmed = enough_activity && enough_strong_hits && large_enough;

                    debug_record_t validation_record = {
                        .type = DEBUG_RECORD_VALIDATION,
                        .confirmed = confirmed,
                        .active_ratio = active_ratio,
                        .strong_hits = candidate.strong_hits,
                        .peak_accel = candidate.peak_vibration,
                        .peak_pga_h = candidate.peak_pga_h,
                        .threshold = threshold
                    };
                    debug_push(&validation_record);

                    if (confirmed) {
                        event_active = true;
                        candidate_active = false;
                        event.sequence = ++event_sequence;
                        event.onset_us = candidate.start_us;
                        event.last_active_us = candidate.last_active_us;
                        event.peak_accel = candidate.peak_vibration;
                        event.peak_pga_h = candidate.peak_pga_h;
                        event.peak_pgv_h = candidate.peak_pgv_h;
                        event.peak_pgv_v = candidate.peak_pgv_v;
                        event.peak_vibration = candidate.peak_vibration;
                        event.vx = candidate.vx;
                        event.vy = candidate.vy;
                        event.vz = candidate.vz;
                        event.cav_g_s = candidate.cav_g_s;

                        memset(&capture, 0, sizeof(capture));
                        prebuffer_copy(capture.h_g, capture.v_g);
                        capture_count = CAPTURE_PRE_SAMPLES;
                        waveform_divider = 0;

                        bool buzzer_queued = buzzer_alert();
                        debug_record_t confirmed_record = {
                            .type = DEBUG_RECORD_FINAL,
                            .confirmed = 1,
                            .buzzer_queued = buzzer_queued,
                            .sequence = event.sequence
                        };
                        debug_push(&confirmed_record);
                    } else {
                        candidate_active = false;
                        waveform_divider = 0;
                        onset_streak = 0;
                        memset(&candidate, 0, sizeof(candidate));
                    }
                }
            }
        }

        debug_snapshot_t snapshot = {
            .vibration = vibration,
            .pga_h = pga_h,
            .pga_v = pga_v,
            .threshold = threshold,
            .amplitude_trigger = amplitude_trigger,
            .onset_trigger = onset_trigger,
            .state = event_active ? STATE_CAPTURE : candidate_active ? STATE_CANDIDATE : STATE_NORMAL,
            .sequence = event_active ? event.sequence : 0,
            .candidate_samples = candidate.samples,
            .candidate_active = candidate.active_count,
            .candidate_strong = candidate.strong_hits,
            .peak_pga = event_active ? event.peak_pga_h : candidate.peak_pga_h,
            .peak_vibration = event_active ? event.peak_vibration : candidate.peak_vibration
        };
        snapshot_publish(&snapshot);

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
    event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(event_capture_t));
    debug_queue = xQueueCreate(DEBUG_QUEUE_LENGTH, sizeof(debug_record_t));

    if (buzzer_queue == NULL || espnow_tx_queue == NULL || event_queue == NULL || debug_queue == NULL) {
        fatal_error(6);
    }

    if (xTaskCreate(buzzer_task, "buzzer_task", BUZZER_TASK_STACK, NULL, BUZZER_TASK_PRIORITY, NULL) != pdPASS) {
        fatal_error(6);
    }
    if (xTaskCreate(espnow_tx_task, "espnow_tx_task", ESPNOW_TX_TASK_STACK, NULL, ESPNOW_TX_TASK_PRIORITY, NULL) != pdPASS) {
        fatal_error(6);
    }
    if (xTaskCreate(analysis_task, "analysis_task", ANALYSIS_TASK_STACK, NULL, ANALYSIS_TASK_PRIORITY, NULL) != pdPASS) {
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
