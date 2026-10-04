#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define I2C_SDA_GPIO                 4
#define I2C_SCL_GPIO                 5
#define BUZZER_GPIO                  10

#define ADXL345_ADDRESS              0x53
#define ADXL345_REG_DEVID            0x00
#define ADXL345_REG_BWRATE           0x2C
#define ADXL345_REG_DATAFMT          0x31
#define ADXL345_REG_POWERCTL         0x2D
#define ADXL345_REG_DATAX0           0x32
#define ADXL345_DEVICE_ID            0xE5

#define SAMPLE_RATE_HZ               100
#define SAMPLE_PERIOD_MS             10

#define CALIBRATION_SECONDS          5
#define CALIBRATION_SAMPLES          \
    (CALIBRATION_SECONDS * SAMPLE_RATE_HZ)

#define GRAVITY_ALPHA                0.005f

#define STA_SAMPLES                  10
#define LTA_SAMPLES                  100
#define DETECTOR_HISTORY_SAMPLES     \
    (STA_SAMPLES + LTA_SAMPLES)

#define STA_LTA_TRIGGER              3.0f

#define MIN_DYNAMIC_G                0.025f
#define NOISE_MULTIPLIER             6.0f

#define VALIDATION_MS                1200
#define VALIDATION_SAMPLES           \
    (VALIDATION_MS / SAMPLE_PERIOD_MS)

#define VALIDATION_MIN_ACTIVE_RATIO  0.35f
#define VALIDATION_MIN_SECOND_ACTIVE 10

#define VALIDATION_MIN_STRONG_HITS   3
#define VALIDATION_STRONG_MULTIPLIER 2.0f

#define VALIDATION_MIN_DIRECTION_CHANGES 4

#define VALIDATION_MAX_QUIET_SAMPLES 40

#define VALIDATION_MIN_PEAK_MULTIPLIER 1.5f

#define EVENT_COOLDOWN_MS            10000

#define WAVEFORM_RATE_HZ             10
#define WAVEFORM_DECIMATION          \
    (SAMPLE_RATE_HZ / WAVEFORM_RATE_HZ)

#define WAVEFORM_PRE_SECONDS         2
#define WAVEFORM_POST_SECONDS        6

#define WAVEFORM_PRE_SAMPLES         \
    (WAVEFORM_PRE_SECONDS * WAVEFORM_RATE_HZ)

#define WAVEFORM_POST_SAMPLES        \
    (WAVEFORM_POST_SECONDS * WAVEFORM_RATE_HZ)

#define WAVEFORM_TOTAL_SAMPLES       \
    (WAVEFORM_PRE_SAMPLES + WAVEFORM_POST_SAMPLES)

#define ASSUMED_SOURCE_DISTANCE_KM   10.0f

#define ADXL345_G_PER_LSB            0.0039f

#define ESPNOW_CHANNEL               1

#define PACKET_VERSION               1

#define MSG_EVENT_START              1
#define MSG_WAVEFORM                 2
#define MSG_EVENT_FINAL              3

#define LEVEL_MILD                   1
#define LEVEL_MODERATE               2
#define LEVEL_STRONG                 3
#define LEVEL_VERY_STRONG            4

#define NODE_ID                      4

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

_Static_assert(
    sizeof(event_packet_t) <= ESP_NOW_MAX_DATA_LEN,
    "event packet too large"
);

_Static_assert(
    sizeof(waveform_packet_t) <= ESP_NOW_MAX_DATA_LEN,
    "waveform packet too large"
);

static const uint8_t master_mac[6] = {
    0x98, 0xF4, 0xAB,
    0x3A, 0x62, 0x00
};

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t adxl_device;

static float gravity_x = 0.0f;
static float gravity_y = 0.0f;
static float gravity_z = 0.0f;

static float noise_rms = 0.0f;

static float detector_history[
    DETECTOR_HISTORY_SAMPLES
];

static int detector_index = 0;
static int detector_count = 0;
static float detector_sum = 0.0f;

static float prebuffer[
    WAVEFORM_PRE_SAMPLES
];

static int prebuffer_index = 0;
static int prebuffer_count = 0;

static uint16_t event_sequence = 0;

static esp_err_t adxl_write_register(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t data[2] = {
        reg,
        value
    };

    return i2c_master_transmit(
        adxl_device,
        data,
        sizeof(data),
        1000
    );
}

static esp_err_t adxl_read_register(
    uint8_t reg,
    uint8_t *data,
    size_t length
)
{
    return i2c_master_transmit_receive(
        adxl_device,
        &reg,
        1,
        data,
        length,
        1000
    );
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

    if (
        i2c_new_master_bus(
            &bus_config,
            &i2c_bus
        ) != ESP_OK
    )
    {
        return false;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADXL345_ADDRESS,
        .scl_speed_hz = 100000
    };

    return i2c_master_bus_add_device(
        i2c_bus,
        &device_config,
        &adxl_device
    ) == ESP_OK;
}

static bool adxl_init(void)
{
    uint8_t device_id = 0;

    if (
        adxl_read_register(
            ADXL345_REG_DEVID,
            &device_id,
            1
        ) != ESP_OK
    )
    {
        return false;
    }

    if (
        device_id !=
        ADXL345_DEVICE_ID
    )
    {
        return false;
    }

    if (
        adxl_write_register(
            ADXL345_REG_DATAFMT,
            0x08
        ) != ESP_OK
    )
    {
        return false;
    }

    if (
        adxl_write_register(
            ADXL345_REG_BWRATE,
            0x0A
        ) != ESP_OK
    )
    {
        return false;
    }

    if (
        adxl_write_register(
            ADXL345_REG_POWERCTL,
            0x08
        ) != ESP_OK
    )
    {
        return false;
    }

    return true;
}

static bool adxl_read_g(
    float *x,
    float *y,
    float *z
)
{
    uint8_t data[6];

    if (
        adxl_read_register(
            ADXL345_REG_DATAX0,
            data,
            6
        ) != ESP_OK
    )
    {
        return false;
    }

    int16_t raw_x =
        (int16_t)(
            ((uint16_t)data[1] << 8) |
            data[0]
        );

    int16_t raw_y =
        (int16_t)(
            ((uint16_t)data[3] << 8) |
            data[2]
        );

    int16_t raw_z =
        (int16_t)(
            ((uint16_t)data[5] << 8) |
            data[4]
        );

    *x =
        raw_x *
        ADXL345_G_PER_LSB;

    *y =
        raw_y *
        ADXL345_G_PER_LSB;

    *z =
        raw_z *
        ADXL345_G_PER_LSB;

    return true;
}

static bool buzzer_init(void)
{
    gpio_config_t config = {
        .pin_bit_mask =
            1ULL << BUZZER_GPIO,

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };

    if (
        gpio_config(
            &config
        ) != ESP_OK
    )
    {
        return false;
    }

    gpio_set_level(
        BUZZER_GPIO,
        0
    );

    return true;
}

static void buzzer_on(void)
{
    gpio_set_level(
        BUZZER_GPIO,
        1
    );
}

static void buzzer_off(void)
{
    gpio_set_level(
        BUZZER_GPIO,
        0
    );
}

static bool espnow_init(void)
{
    wifi_init_config_t wifi_config =
        WIFI_INIT_CONFIG_DEFAULT();

    if (
        esp_wifi_init(
            &wifi_config
        ) != ESP_OK
    )
    {
        return false;
    }

    if (
        esp_wifi_set_mode(
            WIFI_MODE_STA
        ) != ESP_OK
    )
    {
        return false;
    }

    if (
        esp_wifi_start()
        != ESP_OK
    )
    {
        return false;
    }

    if (
        esp_wifi_set_ps(
            WIFI_PS_NONE
        ) != ESP_OK
    )
    {
        return false;
    }

    if (
        esp_wifi_set_channel(
            ESPNOW_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        ) != ESP_OK
    )
    {
        return false;
    }

    if (
        esp_now_init()
        != ESP_OK
    )
    {
        return false;
    }

    esp_now_peer_info_t peer = {0};

    memcpy(
        peer.peer_addr,
        master_mac,
        6
    );

    peer.channel =
        ESPNOW_CHANNEL;

    peer.ifidx =
        WIFI_IF_STA;

    peer.encrypt =
        false;

    if (
        esp_now_add_peer(
            &peer
        ) != ESP_OK
    )
    {
        return false;
    }

    return true;
}

static bool send_packet(
    const uint8_t *data,
    size_t length
)
{
    esp_err_t result =
        esp_now_send(
            master_mac,
            data,
            length
        );

    if (
        result != ESP_OK
    )
    {
        printf(
            "ESP-NOW SEND ERROR: %s\n",
            esp_err_to_name(result)
        );

        return false;
    }

    return true;
}

static void update_gravity(
    float ax,
    float ay,
    float az
)
{
    gravity_x +=
        GRAVITY_ALPHA *
        (ax - gravity_x);

    gravity_y +=
        GRAVITY_ALPHA *
        (ay - gravity_y);

    gravity_z +=
        GRAVITY_ALPHA *
        (az - gravity_z);
}

static void get_dynamic_acceleration(
    float ax,
    float ay,
    float az,
    float *dx,
    float *dy,
    float *dz
)
{
    *dx =
        ax - gravity_x;

    *dy =
        ay - gravity_y;

    *dz =
        az - gravity_z;
}

static float vector_magnitude(
    float x,
    float y,
    float z
)
{
    return sqrtf(
        x * x +
        y * y +
        z * z
    );
}

static float horizontal_magnitude(
    float x,
    float y
)
{
    return sqrtf(
        x * x +
        y * y
    );
}

static void detector_reset(void)
{
    memset(
        detector_history,
        0,
        sizeof(detector_history)
    );

    detector_index = 0;
    detector_count = 0;
    detector_sum = 0.0f;
}

static float detector_update(
    float vibration
)
{
    float squared =
        vibration *
        vibration;

    detector_sum -=
        detector_history[
            detector_index
        ];

    detector_history[
        detector_index
    ] = squared;

    detector_sum +=
        squared;

    detector_index++;

    if (
        detector_index >=
        DETECTOR_HISTORY_SAMPLES
    )
    {
        detector_index = 0;
    }

    if (
        detector_count <
        DETECTOR_HISTORY_SAMPLES
    )
    {
        detector_count++;
    }

    if (
        detector_count <
        DETECTOR_HISTORY_SAMPLES
    )
    {
        return 0.0f;
    }

    int newest =
        detector_index - 1;

    if (
        newest < 0
    )
    {
        newest =
            DETECTOR_HISTORY_SAMPLES - 1;
    }

    float sta_sum = 0.0f;

    for (
        int i = 0;
        i < STA_SAMPLES;
        i++
    )
    {
        int index =
            newest - i;

        while (
            index < 0
        )
        {
            index +=
                DETECTOR_HISTORY_SAMPLES;
        }

        sta_sum +=
            detector_history[index];
    }

    float lta_sum =
        detector_sum -
        sta_sum;

    float sta_rms =
        sqrtf(
            sta_sum /
            STA_SAMPLES
        );

    float lta_rms =
        sqrtf(
            lta_sum /
            LTA_SAMPLES
        );

    if (
        lta_rms <
        0.000001f
    )
    {
        return 0.0f;
    }

    return sta_rms /
           lta_rms;
}

static void prebuffer_add(
    float vibration
)
{
    prebuffer[
        prebuffer_index
    ] = vibration;

    prebuffer_index++;

    if (
        prebuffer_index >=
        WAVEFORM_PRE_SAMPLES
    )
    {
        prebuffer_index = 0;
    }

    if (
        prebuffer_count <
        WAVEFORM_PRE_SAMPLES
    )
    {
        prebuffer_count++;
    }
}

static void prebuffer_copy(
    float *destination
)
{
    int start =
        prebuffer_index;

    for (
        int i = 0;
        i < WAVEFORM_PRE_SAMPLES;
        i++
    )
    {
        destination[i] =
            prebuffer[
                (start + i) %
                WAVEFORM_PRE_SAMPLES
            ];
    }
}

static float get_detection_threshold(void)
{
    float threshold =
        noise_rms *
        NOISE_MULTIPLIER;

    if (
        threshold <
        MIN_DYNAMIC_G
    )
    {
        threshold =
            MIN_DYNAMIC_G;
    }

    return threshold;
}

static int get_level(
    float pga_g
)
{
    if (
        pga_g >=
        0.30f
    )
    {
        return LEVEL_VERY_STRONG;
    }

    if (
        pga_g >=
        0.15f
    )
    {
        return LEVEL_STRONG;
    }

    if (
        pga_g >=
        0.05f
    )
    {
        return LEVEL_MODERATE;
    }

    return LEVEL_MILD;
}

static const char *level_string(
    int level
)
{
    switch (
        level
    )
    {
        case LEVEL_MILD:
            return "MILD";

        case LEVEL_MODERATE:
            return "MODERATE";

        case LEVEL_STRONG:
            return "STRONG";

        case LEVEL_VERY_STRONG:
            return "VERY STRONG";

        default:
            return "UNKNOWN";
    }
}

static float estimate_magnitude(
    float pga_g
)
{
    if (
        pga_g <=
        0.000001f
    )
    {
        return 0.0f;
    }

    float magnitude =
        (
            logf(pga_g) +
            logf(
                ASSUMED_SOURCE_DISTANCE_KM +
                7.28f
            ) +
            2.501f
        ) /
        0.623f;

    if (
        magnitude <
        0.0f
    )
    {
        magnitude =
            0.0f;
    }

    if (
        magnitude >
        9.9f
    )
    {
        magnitude =
            9.9f;
    }

    return magnitude;
}

static int estimate_intensity(
    float pga_g
)
{
    if (
        pga_g <=
        0.0f
    )
    {
        return 0;
    }

    float pga_percent_g =
        pga_g *
        100.0f;

    float intensity =
        3.66f *
        log10f(
            pga_percent_g
        ) +
        1.99f;

    int value =
        (int)lroundf(
            intensity
        );

    if (
        value < 1
    )
    {
        value = 1;
    }

    if (
        value > 8
    )
    {
        value = 8;
    }

    return value;
}

static const char *intensity_string(
    int intensity
)
{
    switch (
        intensity
    )
    {
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

        default:
            return "UNKNOWN";
    }
}

static bool calibrate_sensor(void)
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    for (
        int i = 0;
        i < CALIBRATION_SAMPLES;
        i++
    )
    {
        float ax;
        float ay;
        float az;

        if (
            !adxl_read_g(
                &ax,
                &ay,
                &az
            )
        )
        {
            return false;
        }

        sum_x += ax;
        sum_y += ay;
        sum_z += az;

        vTaskDelay(
            pdMS_TO_TICKS(
                SAMPLE_PERIOD_MS
            )
        );
    }

    gravity_x =
        sum_x /
        CALIBRATION_SAMPLES;

    gravity_y =
        sum_y /
        CALIBRATION_SAMPLES;

    gravity_z =
        sum_z /
        CALIBRATION_SAMPLES;

    float noise_sum =
        0.0f;

    for (
        int i = 0;
        i < CALIBRATION_SAMPLES;
        i++
    )
    {
        float ax;
        float ay;
        float az;

        if (
            !adxl_read_g(
                &ax,
                &ay,
                &az
            )
        )
        {
            return false;
        }

        float dx =
            ax - gravity_x;

        float dy =
            ay - gravity_y;

        float dz =
            az - gravity_z;

        float motion =
            vector_magnitude(
                dx,
                dy,
                dz
            );

        noise_sum +=
            motion *
            motion;

        vTaskDelay(
            pdMS_TO_TICKS(
                SAMPLE_PERIOD_MS
            )
        );
    }

    noise_rms =
        sqrtf(
            noise_sum /
            CALIBRATION_SAMPLES
        );

    detector_reset();

    memset(
        prebuffer,
        0,
        sizeof(prebuffer)
    );

    prebuffer_index =
        0;

    prebuffer_count =
        0;

    return true;
}

static void send_event_start(
    uint16_t sequence,
    float trigger_pga
)
{
    event_packet_t packet = {
        .version =
            PACKET_VERSION,

        .type =
            MSG_EVENT_START,

        .node_id =
            NODE_ID,

        .level =
            LEVEL_MILD,

        .intensity =
            0,

        .reserved =
            0,

        .sequence =
            sequence,

        .peak_acceleration_g =
            trigger_pga,

        .peak_vibration_g =
            0.0f,

        .rms_vibration_g =
            0.0f,

        .duration_s =
            0.0f,

        .magnitude_est =
            0.0f
    };

    send_packet(
        (const uint8_t *)&packet,
        sizeof(packet)
    );
}

static void send_waveform(
    uint16_t sequence,
    const float *waveform
)
{
    waveform_packet_t packet = {
        .version =
            PACKET_VERSION,

        .type =
            MSG_WAVEFORM,

        .node_id =
            NODE_ID,

        .reserved1 =
            0,

        .sample_count =
            WAVEFORM_TOTAL_SAMPLES,

        .reserved2 =
            0,

        .sequence =
            sequence
    };

    for (
        int i = 0;
        i < WAVEFORM_TOTAL_SAMPLES;
        i++
    )
    {
        float scaled =
            waveform[i] *
            1000.0f;

        if (
            scaled >
            32767.0f
        )
        {
            scaled =
                32767.0f;
        }

        if (
            scaled <
            -32768.0f
        )
        {
            scaled =
                -32768.0f;
        }

        packet.samples[i] =
            (int16_t)lroundf(
                scaled
            );
    }

    send_packet(
        (const uint8_t *)&packet,
        sizeof(packet)
    );
}

static void send_event_final(
    uint16_t sequence,
    float peak_pga,
    float peak_vibration,
    float rms_vibration,
    float duration
)
{
    int level =
        get_level(
            peak_pga
        );

    int intensity =
        estimate_intensity(
            peak_pga
        );

    float magnitude =
        estimate_magnitude(
            peak_pga
        );

    event_packet_t packet = {
        .version =
            PACKET_VERSION,

        .type =
            MSG_EVENT_FINAL,

        .node_id =
            NODE_ID,

        .level =
            level,

        .intensity =
            intensity,

        .reserved =
            0,

        .sequence =
            sequence,

        .peak_acceleration_g =
            peak_pga,

        .peak_vibration_g =
            peak_vibration,

        .rms_vibration_g =
            rms_vibration,

        .duration_s =
            duration,

        .magnitude_est =
            magnitude
    };

    send_packet(
        (const uint8_t *)&packet,
        sizeof(packet)
    );
}

static void halt(void)
{
    while (1)
    {
        vTaskDelay(
            pdMS_TO_TICKS(
                1000
            )
        );
    }
}

void app_main(void)
{
    esp_log_level_set(
        "*",
        ESP_LOG_NONE
    );

    esp_err_t nvs_err =
        nvs_flash_init();

    if (
        nvs_err ==
        ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err ==
        ESP_ERR_NVS_NEW_VERSION_FOUND
    )
    {
        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ESP_ERROR_CHECK(
            nvs_flash_init()
        );
    }

    if (
        esp_netif_init()
        != ESP_OK
    )
    {
        halt();
    }

    esp_err_t event_err =
        esp_event_loop_create_default();

    if (
        event_err != ESP_OK &&
        event_err != ESP_ERR_INVALID_STATE
    )
    {
        halt();
    }

    if (
        !i2c_init()
    )
    {
        printf(
            "I2C INIT FAILED\n"
        );

        halt();
    }

    if (
        !adxl_init()
    )
    {
        printf(
            "ADXL345 INIT FAILED\n"
        );

        halt();
    }

    if (
        !buzzer_init()
    )
    {
        printf(
            "BUZZER INIT FAILED\n"
        );

        halt();
    }

    if (
        !espnow_init()
    )
    {
        printf(
            "ESP-NOW INIT FAILED\n"
        );

        halt();
    }

    printf(
        "\n"
        "================================\n"
        "EARTHQUAKE ALERT SLAVE\n"
        "================================\n"
        "NODE ID:           %d\n"
        "CHANNEL:           %d\n"
        "MASTER:            %02X:%02X:%02X:%02X:%02X:%02X\n"
        "SAMPLE RATE:       %d Hz\n"
        "VALIDATION:        %d ms\n"
        "WAVEFORM RATE:     %d Hz\n"
        "WAVEFORM RANGE:    -%d s to +%d s\n"
        "================================\n\n",
        NODE_ID,
        ESPNOW_CHANNEL,
        master_mac[0],
        master_mac[1],
        master_mac[2],
        master_mac[3],
        master_mac[4],
        master_mac[5],
        SAMPLE_RATE_HZ,
        VALIDATION_MS,
        WAVEFORM_RATE_HZ,
        WAVEFORM_PRE_SECONDS,
        WAVEFORM_POST_SECONDS
    );

    printf(
        "Keep sensor still during calibration.\n"
    );

    printf(
        "CALIBRATING...\n"
    );

    if (
        !calibrate_sensor()
    )
    {
        printf(
            "CALIBRATION FAILED\n"
        );

        halt();
    }

    float detection_threshold =
        get_detection_threshold();

    printf(
        "\n"
        "CALIBRATION COMPLETE\n"
        "GRAVITY: X=%.4f Y=%.4f Z=%.4f g\n"
        "NOISE RMS: %.5f g\n"
        "DETECTION THRESHOLD: %.5f g\n"
        "READY\n\n",
        gravity_x,
        gravity_y,
        gravity_z,
        noise_rms,
        detection_threshold
    );

    bool candidate_active =
        false;

    bool event_active =
        false;

    int candidate_samples =
        0;

    int candidate_active_count =
        0;

    int candidate_second_active =
        0;

    int candidate_strong_hits =
        0;

    int candidate_quiet_count =
        0;

    int candidate_max_quiet =
        0;

    int candidate_direction_changes =
        0;

    float candidate_peak_vibration =
        0.0f;

    float candidate_peak_pga =
        0.0f;

    float previous_dx =
        0.0f;

    float previous_dy =
        0.0f;

    float previous_dz =
        0.0f;

    bool previous_direction_valid =
        false;

    int64_t candidate_start_us =
        0;

    int64_t event_start_us =
        0;

    int64_t last_event_us =
        -EVENT_COOLDOWN_MS *
        1000LL;

    float waveform[
        WAVEFORM_TOTAL_SAMPLES
    ];

    int waveform_count =
        0;

    int waveform_divider =
        0;

    uint16_t current_sequence =
        0;

    float peak_pga =
        0.0f;

    float peak_vibration =
        0.0f;

    float rms_sum_squared =
        0.0f;

    uint32_t rms_samples =
        0;

    int64_t debug_timer_us =
        0;

    while (1)
    {
        int64_t now_us =
            esp_timer_get_time();

        float ax;
        float ay;
        float az;

        if (
            !adxl_read_g(
                &ax,
                &ay,
                &az
            )
        )
        {
            vTaskDelay(
                pdMS_TO_TICKS(
                    SAMPLE_PERIOD_MS
                )
            );

            continue;
        }

        float dx;
        float dy;
        float dz;

        get_dynamic_acceleration(
            ax,
            ay,
            az,
            &dx,
            &dy,
            &dz
        );

        float vibration =
            vector_magnitude(
                dx,
                dy,
                dz
            );

        float pga =
            horizontal_magnitude(
                dx,
                dy
            );

        float sta_lta =
            detector_update(
                vibration
            );

        float current_threshold =
            get_detection_threshold();

        bool amplitude_trigger =
            vibration >=
            current_threshold;

        bool onset_trigger =
            amplitude_trigger &&
            sta_lta >=
            STA_LTA_TRIGGER;

        if (
            now_us -
            debug_timer_us >=
            1000000LL
        )
        {
            printf(
                "DEBUG | VIB=%.4f g | PGA=%.4f g | TH=%.4f g | STA/LTA=%.2f | AMP=%d | ONSET=%d | STATE=%s\n",
                vibration,
                pga,
                current_threshold,
                sta_lta,
                amplitude_trigger,
                onset_trigger,
                candidate_active ?
                    "CANDIDATE" :
                    event_active ?
                    "EVENT" :
                    "NORMAL"
            );

            debug_timer_us =
                now_us;
        }

        waveform_divider++;

        bool waveform_sample_due =
            false;

        if (
            waveform_divider >=
            WAVEFORM_DECIMATION
        )
        {
            waveform_divider = 0;
            waveform_sample_due = true;
        }

        if (
            !candidate_active &&
            !event_active
        )
        {
            prebuffer_add(
                vibration
            );
        }

        if (
            !candidate_active &&
            !event_active
        )
        {
            if (
                onset_trigger &&
                now_us -
                last_event_us >=
                EVENT_COOLDOWN_MS *
                1000LL &&
                prebuffer_count >=
                WAVEFORM_PRE_SAMPLES
            )
            {
                candidate_active =
                    true;

                candidate_samples =
                    0;

                candidate_active_count =
                    0;

                candidate_second_active =
                    0;

                candidate_strong_hits =
                    0;

                candidate_quiet_count =
                    0;

                candidate_max_quiet =
                    0;

                candidate_direction_changes =
                    0;

                candidate_peak_vibration =
                    vibration;

                candidate_peak_pga =
                    pga;

                candidate_start_us =
                    now_us;

                previous_dx =
                    dx;

                previous_dy =
                    dy;

                previous_dz =
                    dz;

                previous_direction_valid =
                    true;

                prebuffer_copy(
                    waveform
                );

                waveform_count =
                    WAVEFORM_PRE_SAMPLES;

                waveform_divider =
                    0;

                printf(
                    "\n"
                    "CANDIDATE EVENT\n"
                    "Validating for %d ms...\n\n",
                    VALIDATION_MS
                );
            }
        }

        if (
            candidate_active
        )
        {
            candidate_samples++;

            if (
                amplitude_trigger
            )
            {
                candidate_active_count++;
                candidate_quiet_count = 0;
            }
            else
            {
                candidate_quiet_count++;

                if (
                    candidate_quiet_count >
                    candidate_max_quiet
                )
                {
                    candidate_max_quiet =
                        candidate_quiet_count;
                }
            }

            if (
                candidate_samples >
                VALIDATION_SAMPLES / 2
            )
            {
                if (
                    amplitude_trigger
                )
                {
                    candidate_second_active++;
                }
            }

            if (
                vibration >
                candidate_peak_vibration
            )
            {
                candidate_peak_vibration =
                    vibration;
            }

            if (
                pga >
                candidate_peak_pga
            )
            {
                candidate_peak_pga =
                    pga;
            }

            if (
                vibration >=
                current_threshold *
                VALIDATION_STRONG_MULTIPLIER
            )
            {
                candidate_strong_hits++;
            }

            if (
                previous_direction_valid
            )
            {
                float dot =
                    dx * previous_dx +
                    dy * previous_dy +
                    dz * previous_dz;

                float current_mag =
                    vector_magnitude(
                        dx,
                        dy,
                        dz
                    );

                float previous_mag =
                    vector_magnitude(
                        previous_dx,
                        previous_dy,
                        previous_dz
                    );

                if (
                    current_mag >=
                    current_threshold &&
                    previous_mag >=
                    current_threshold &&
                    dot < 0.0f
                )
                {
                    candidate_direction_changes++;
                }
            }

            previous_dx =
                dx;

            previous_dy =
                dy;

            previous_dz =
                dz;

            previous_direction_valid =
                true;

            if (
                waveform_sample_due &&
                waveform_count <
                WAVEFORM_TOTAL_SAMPLES
            )
            {
                waveform[
                    waveform_count
                ] = vibration;

                waveform_count++;
            }

            if (
                candidate_samples >=
                VALIDATION_SAMPLES
            )
            {
                float active_ratio =
                    (
                        float
                    )candidate_active_count /
                    VALIDATION_SAMPLES;

                bool enough_activity =
                    active_ratio >=
                    VALIDATION_MIN_ACTIVE_RATIO;

                bool activity_continues =
                    candidate_second_active >=
                    VALIDATION_MIN_SECOND_ACTIVE;

                bool multiple_strong_samples =
                    candidate_strong_hits >=
                    VALIDATION_MIN_STRONG_HITS;

                bool direction_is_dynamic =
                    candidate_direction_changes >=
                    VALIDATION_MIN_DIRECTION_CHANGES;

                bool quiet_gap_ok =
                    candidate_max_quiet <=
                    VALIDATION_MAX_QUIET_SAMPLES;

                bool large_enough =
                    candidate_peak_vibration >=
                    current_threshold *
                    VALIDATION_MIN_PEAK_MULTIPLIER;

                bool confirmed =
                    enough_activity &&
                    activity_continues &&
                    multiple_strong_samples &&
                    direction_is_dynamic &&
                    quiet_gap_ok &&
                    large_enough;

                printf(
                    "\n"
                    "VALIDATION RESULT\n"
                    "Active ratio:       %.2f\n"
                    "Second-half active: %d\n"
                    "Strong hits:        %d\n"
                    "Direction changes:  %d\n"
                    "Max quiet gap:      %d samples\n"
                    "Peak vibration:     %.4f g\n"
                    "Peak PGA:           %.4f g\n"
                    "Threshold:          %.4f g\n",
                    active_ratio,
                    candidate_second_active,
                    candidate_strong_hits,
                    candidate_direction_changes,
                    candidate_max_quiet,
                    candidate_peak_vibration,
                    candidate_peak_pga,
                    current_threshold
                );

                if (
                    !confirmed
                )
                {
                    printf(
                        "FALSE TRIGGER - DISCARDED\n\n"
                    );

                    candidate_active =
                        false;

                    waveform_count =
                        0;

                    waveform_divider =
                        0;

                    candidate_samples =
                        0;

                    candidate_active_count =
                        0;

                    candidate_second_active =
                        0;

                    candidate_strong_hits =
                        0;

                    candidate_quiet_count =
                        0;

                    candidate_max_quiet =
                        0;

                    candidate_direction_changes =
                        0;

                    candidate_peak_vibration =
                        0.0f;

                    candidate_peak_pga =
                        0.0f;

                    previous_direction_valid =
                        false;

                    detector_reset();

                    continue;
                }

                event_active =
                    true;

                candidate_active =
                    false;

                event_start_us =
                    candidate_start_us;

                current_sequence =
                    ++event_sequence;

                peak_pga =
                    candidate_peak_pga;

                peak_vibration =
                    candidate_peak_vibration;

                rms_sum_squared =
                    candidate_peak_vibration *
                    candidate_peak_vibration;

                rms_samples =
                    1;

                buzzer_on();

                printf(
                    "\n"
                    "################################\n"
                    "EARTHQUAKE CONFIRMED\n"
                    "################################\n\n"
                );

                send_event_start(
                    current_sequence,
                    candidate_peak_pga
                );
            }
        }

        if (
            event_active
        )
        {
            if (
                pga >
                peak_pga
            )
            {
                peak_pga =
                    pga;
            }

            if (
                vibration >
                peak_vibration
            )
            {
                peak_vibration =
                    vibration;
            }

            rms_sum_squared +=
                vibration *
                vibration;

            rms_samples++;

            if (
                waveform_sample_due &&
                waveform_count <
                WAVEFORM_TOTAL_SAMPLES
            )
            {
                waveform[
                    waveform_count
                ] = vibration;

                waveform_count++;
            }

            if (
                waveform_count >=
                WAVEFORM_TOTAL_SAMPLES
            )
            {
                float rms_vibration =
                    sqrtf(
                        rms_sum_squared /
                        (
                            rms_samples >
                            0
                            ?
                            rms_samples
                            :
                            1
                        )
                    );

                float duration =
                    (
                        now_us -
                        event_start_us
                    ) /
                    1000000.0f;

                int level =
                    get_level(
                        peak_pga
                    );

                int intensity =
                    estimate_intensity(
                        peak_pga
                    );

                float magnitude =
                    estimate_magnitude(
                        peak_pga
                    );

                printf(
                    "\n"
                    "================================\n"
                    "EARTHQUAKE DETECTED\n"
                    "================================\n"
                    "LEVEL:            %s\n"
                    "PEAK PGA:         %.4f g\n"
                    "PEAK VIBRATION:   %.4f g\n"
                    "RMS VIBRATION:    %.4f g\n"
                    "DURATION:         %.2f s\n"
                    "MAGNITUDE EST:    %.1f\n"
                    "INTENSITY EST:    %s\n"
                    "SEQUENCE:         %u\n"
                    "================================\n\n",
                    level_string(
                        level
                    ),
                    peak_pga,
                    peak_vibration,
                    rms_vibration,
                    duration,
                    magnitude,
                    intensity_string(
                        intensity
                    ),
                    current_sequence
                );

                buzzer_off();

                vTaskDelay(
                    pdMS_TO_TICKS(50)
                );

                send_waveform(
                    current_sequence,
                    waveform
                );

                vTaskDelay(
                    pdMS_TO_TICKS(50)
                );

                send_event_final(
                    current_sequence,
                    peak_pga,
                    peak_vibration,
                    rms_vibration,
                    duration
                );

                printf(
                    "EVENT TRANSMISSION COMPLETE\n\n"
                );

                event_active =
                    false;

                last_event_us =
                    now_us;

                waveform_count =
                    0;

                waveform_divider =
                    0;

                peak_pga =
                    0.0f;

                peak_vibration =
                    0.0f;

                rms_sum_squared =
                    0.0f;

                rms_samples =
                    0;

                previous_direction_valid =
                    false;

                detector_reset();

                detection_threshold =
                    get_detection_threshold();
            }
        }

        update_gravity(
            ax,
            ay,
            az
        );

        vTaskDelay(
            pdMS_TO_TICKS(
                SAMPLE_PERIOD_MS
            )
        );
    }
}