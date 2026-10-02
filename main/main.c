#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_now.h"
#include "nvs_flash.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define I2C_SDA_GPIO            4
#define I2C_SCL_GPIO            5
#define BUZZER_GPIO             10

#define WIFI_SSID               "FREE WIFI"
#define WIFI_PASSWORD           "11111111"

#define ADXL345_ADDRESS         0x53
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_BWRATE      0x2C
#define ADXL345_REG_DATAFMT     0x31
#define ADXL345_REG_POWERCTL    0x2D
#define ADXL345_REG_DATAX0      0x32
#define ADXL345_DEVICE_ID       0xE5

#define SAMPLE_RATE_HZ          100
#define SAMPLE_PERIOD_MS        10
#define CALIBRATION_SECONDS     5

#define FILTER_ALPHA            0.01f

#define EARTHQUAKE_THRESHOLD_G  0.08f
#define REQUIRED_HITS           8
#define HIT_WINDOW              10

#define EVENT_ANALYSIS_MS       3000
#define EVENT_COOLDOWN_MS       10000

#define ASSUMED_SOURCE_DISTANCE_KM 10.0f

#define ADXL345_G_PER_LSB       0.0039f

/* ---- Hub link config ---- */
#ifndef NODE_ID
#define NODE_ID                 4          /* unique, 1..7 */
#endif
#define NODE_LAT                14.676000f
#define NODE_LNG                121.043700f
#define ESPNOW_CHANNEL          1          /* must equal hub WIFI_CHANNEL */
#define HAZARD_NONE             0
#define HAZARD_EARTHQUAKE       5
#define HEARTBEAT_MS            1000
#define ALERT_HOLD_MS           30000

typedef struct {
    int   nodeID;
    float locationLat;
    float locationLng;
    int   hazardType;
    int   severityLevel;
} disaster_message_t;

typedef struct { int nodeID; } ack_message_t;

_Static_assert(sizeof(disaster_message_t) == 20, "struct must match hub layout");

static const uint8_t bcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static volatile bool hub_acked = false;
static volatile bool wifi_reconnect_enabled = true;
static bool sntp_started = false;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t adxl_device;

static float lowpass_x;
static float lowpass_y;
static float lowpass_z;

static EventGroupHandle_t wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0

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

    if (i2c_new_master_bus(&bus_config, &i2c_bus) != ESP_OK) return false;

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_ADDRESS,
        .scl_speed_hz = 100000
    };

    return i2c_master_bus_add_device(i2c_bus, &device_config, &adxl_device) == ESP_OK;
}

static bool adxl_init(void)
{
    uint8_t device_id = 0;

    if (adxl_read_register(ADXL345_REG_DEVID, &device_id, 1) != ESP_OK) return false;
    if (device_id != ADXL345_DEVICE_ID) return false;
    if (adxl_write_register(ADXL345_REG_DATAFMT, 0x08) != ESP_OK) return false;
    if (adxl_write_register(ADXL345_REG_BWRATE, 0x0A) != ESP_OK) return false;
    if (adxl_write_register(ADXL345_REG_POWERCTL, 0x08) != ESP_OK) return false;

    return true;
}

static esp_err_t adxl_read_raw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    esp_err_t err = adxl_read_register(ADXL345_REG_DATAX0, data, 6);
    if (err != ESP_OK) return err;

    *x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    *y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    *z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    return ESP_OK;
}

static bool buzzer_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    if (gpio_config(&config) != ESP_OK) return false;
    gpio_set_level(BUZZER_GPIO, 0);
    return true;
}

static void buzzer_on(void)  { gpio_set_level(BUZZER_GPIO, 1); }
static void buzzer_off(void) { gpio_set_level(BUZZER_GPIO, 0); }

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        if (wifi_reconnect_enabled) esp_wifi_connect();
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Starts the Wi-Fi driver in STA mode and tries to join the router. Returns true if the driver started. */
static bool wifi_start(bool *connected)
{
    *connected = false;

    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) return false;

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;

    if (esp_netif_create_default_wifi_sta() == NULL) return false;

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&init_cfg) != ESP_OK) return false;

    if (esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL) != ESP_OK) return false;
    if (esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                            &wifi_event_handler, NULL, NULL) != ESP_OK) return false;

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) return false;
    if (esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK) return false;
    if (esp_wifi_start() != ESP_OK) return false;

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    *connected = (bits & WIFI_CONNECTED_BIT) != 0;
    return true;
}

/* Best effort. A failure only leaves the printed timestamp unsynced. */
static bool time_sync(void)
{
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");

    if (esp_netif_sntp_init(&sntp_config) != ESP_OK) return false;
    sntp_started = true;

    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(15000)) != ESP_OK) return false;

    return time(NULL) >= 1700000000;
}

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len != (int)sizeof(ack_message_t)) return;
    ack_message_t a;
    memcpy(&a, data, sizeof(a));
    if (a.nodeID == NODE_ID) hub_acked = true;
}

static bool espnow_start(void)
{
    if (sntp_started) {
        esp_netif_sntp_deinit();
        sntp_started = false;
    }

    wifi_reconnect_enabled = false;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;

    uint8_t primary;
    wifi_second_chan_t second;
    if (esp_wifi_get_channel(&primary, &second) != ESP_OK || primary != ESPNOW_CHANNEL) return false;

    if (esp_now_init() != ESP_OK) return false;
    if (esp_now_register_recv_cb(espnow_recv_cb) != ESP_OK) return false;

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, bcast_addr, 6);
    peer.channel = 0;                 /* 0 = use the current channel */
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    return esp_now_add_peer(&peer) == ESP_OK;
}

static void hub_send(int hazard, int severity)
{
    disaster_message_t m = {
        .nodeID = NODE_ID,
        .locationLat = NODE_LAT,
        .locationLng = NODE_LNG,
        .hazardType = hazard,
        .severityLevel = severity
    };
    esp_now_send(bcast_addr, (const uint8_t *)&m, sizeof(m));
}

static int severity_from_mmi(int mmi)
{
    if (mmi >= 8) return 3;
    if (mmi >= 6) return 2;
    return 1;
}

static bool calibrate_sensor(void)
{
    const int samples = CALIBRATION_SECONDS * SAMPLE_RATE_HZ;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    for (int i = 0; i < samples; i++) {
        int16_t rx, ry, rz;
        if (adxl_read_raw(&rx, &ry, &rz) != ESP_OK) return false;

        sum_x += rx * ADXL345_G_PER_LSB;
        sum_y += ry * ADXL345_G_PER_LSB;
        sum_z += rz * ADXL345_G_PER_LSB;

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    lowpass_x = sum_x / samples;
    lowpass_y = sum_y / samples;
    lowpass_z = sum_z / samples;
    return true;
}

static float calculate_vibration(float ax, float ay, float az)
{
    lowpass_x += FILTER_ALPHA * (ax - lowpass_x);
    lowpass_y += FILTER_ALPHA * (ay - lowpass_y);
    lowpass_z += FILTER_ALPHA * (az - lowpass_z);

    float dx = ax - lowpass_x;
    float dy = ay - lowpass_y;
    float dz = az - lowpass_z;

    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static bool earthquake_detected(float vibration)
{
    static int hit_count = 0;
    static int sample_count = 0;

    sample_count++;
    if (vibration >= EARTHQUAKE_THRESHOLD_G) hit_count++;

    if (sample_count >= HIT_WINDOW) {
        bool detected = hit_count >= REQUIRED_HITS;
        sample_count = 0;
        hit_count = 0;
        return detected;
    }

    return false;
}

static float estimate_magnitude(float pga_g)
{
    if (pga_g <= 0.000001f) return 0.0f;

    float magnitude = (logf(pga_g) + logf(ASSUMED_SOURCE_DISTANCE_KM + 7.28f) + 2.501f) / 0.623f;

    if (magnitude < 0.0f) magnitude = 0.0f;
    if (magnitude > 9.9f) magnitude = 9.9f;
    return magnitude;
}

static int estimate_intensity(float pga_g)
{
    if (pga_g <= 0.0f) return 0;

    float intensity = 3.66f * log10f(pga_g * 100.0f) + 1.99f;
    int mmi = (int)lroundf(intensity);

    if (mmi < 5) mmi = 5;
    if (mmi > 8) mmi = 8;
    return mmi;
}

static const char *mmi_string(int intensity)
{
    switch (intensity) {
        case 1:  return "I";
        case 2:  return "II";
        case 3:  return "III";
        case 4:  return "IV";
        case 5:  return "V";
        case 6:  return "VI";
        case 7:  return "VII";
        case 8:  return "VIII";
        case 9:  return "IX";
        case 10: return "X";
        default: return "UNKNOWN";
    }
}

static void halt(void)
{
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    if (!i2c_init())    halt();
    if (!buzzer_init()) halt();
    if (!adxl_init())   halt();

    bool connected = false;
    if (!wifi_start(&connected)) halt();
    if (connected) time_sync();
    if (!espnow_start()) halt();

    if (!calibrate_sensor()) halt();

    int64_t last_event_time_us = -EVENT_COOLDOWN_MS * 1000LL;
    bool event_active = false;
    int64_t event_start_us = 0;
    time_t event_epoch = 0;

    float peak_vibration = 0.0f;
    float peak_acceleration = 0.0f;
    float peak_pga = 0.0f;

    int alert_severity = 0;
    int64_t alert_until_us = 0;
    int64_t last_hb_us = 0;
    bool ack_reported = false;

    while (1) {
        int64_t now_us = esp_timer_get_time();

        if (alert_severity && now_us > alert_until_us) alert_severity = 0;

        if (now_us - last_hb_us >= HEARTBEAT_MS * 1000LL) {
            hub_send(alert_severity ? HAZARD_EARTHQUAKE : HAZARD_NONE, alert_severity);
            last_hb_us = now_us;
        }

        if (hub_acked && !ack_reported) {
            printf("\nHUB ACK: help is on the way\n\n");
            ack_reported = true;
        }

        int16_t raw_x, raw_y, raw_z;

        if (adxl_read_raw(&raw_x, &raw_y, &raw_z) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        float ax = raw_x * ADXL345_G_PER_LSB;
        float ay = raw_y * ADXL345_G_PER_LSB;
        float az = raw_z * ADXL345_G_PER_LSB;

        float vibration = calculate_vibration(ax, ay, az);
        float total_acceleration = sqrtf(ax * ax + ay * ay + az * az);

        if (!event_active) {
            if (earthquake_detected(vibration) &&
                (now_us - last_event_time_us) >= EVENT_COOLDOWN_MS * 1000LL) {

                event_active = true;
                event_start_us = now_us;
                event_epoch = time(NULL);

                peak_vibration = vibration;
                peak_acceleration = total_acceleration;
                peak_pga = vibration;

                buzzer_on();

                alert_severity = 2;
                alert_until_us = now_us + (EVENT_ANALYSIS_MS + ALERT_HOLD_MS) * 1000LL;
                hub_acked = false;
                ack_reported = false;
                hub_send(HAZARD_EARTHQUAKE, alert_severity);
                last_hb_us = now_us;
            }
        }

        if (event_active) {
            if (vibration > peak_vibration) peak_vibration = vibration;
            if (total_acceleration > peak_acceleration) peak_acceleration = total_acceleration;
            if (vibration > peak_pga) peak_pga = vibration;

            if (now_us - event_start_us >= EVENT_ANALYSIS_MS * 1000LL) {
                float magnitude = estimate_magnitude(peak_pga);
                int intensity = estimate_intensity(peak_pga);

                printf("\n"
                       "EARTHQUAKE DETECTED\n"
                       "TIME: %lld\n"
                       "MAGNITUDE: %.1f\n"
                       "INTENSITY: %s\n"
                       "ACCELERATION: %.3f g\n"
                       "VIBRATION: %.3f g\n"
                       "\n",
                       (long long)event_epoch,
                       magnitude,
                       mmi_string(intensity),
                       peak_acceleration,
                       peak_vibration);

                buzzer_off();

                alert_severity = severity_from_mmi(intensity);
                alert_until_us = now_us + ALERT_HOLD_MS * 1000LL;
                hub_send(HAZARD_EARTHQUAKE, alert_severity);
                last_hb_us = now_us;

                event_active = false;
                last_event_time_us = now_us;

                peak_vibration = 0.0f;
                peak_acceleration = 0.0f;
                peak_pga = 0.0f;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}