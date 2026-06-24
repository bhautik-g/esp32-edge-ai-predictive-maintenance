#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"

#include "driver/i2c.h"
#include "driver/i2s_std.h"

#include "esp_log.h"

#include "owb.h"
#include "owb_gpio.h"
#include "ds18b20.h"

#include "decision_tree_model.h"

static const char *TAG = "ML";
static const char *LABEL = "IMBALANCE_SPEED2";

// ======================================================
// MPU6050 CONFIG
// ======================================================

#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define MPU6050_ADDR                0x68
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B


// ======================================================
// INMP441 CONFIG
// ======================================================

#define I2S_WS      4
#define I2S_SCK     5
#define I2S_SD      6

#define SAMPLE_RATE 16000
#define BUFFER_SIZE 1024

// ======================================================
// DS18B20 CONFIG
// ======================================================

#define GPIO_DS18B20_0  GPIO_NUM_13
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD        (1000)   // milliseconds


// ======================================================
// I2C INIT
// ======================================================

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);

    return i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        0,
        0,
        0
    );
}


// ======================================================
// MPU6050 INIT
// ======================================================

void mpu6050_init()
{
    uint8_t data[2];

    data[0] = MPU6050_PWR_MGMT_1;
    data[1] = 0;

    i2c_master_write_to_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        data,
        2,
        pdMS_TO_TICKS(100)
    );
}


// ======================================================
// READ MPU6050
// ======================================================

void mpu6050_read_accel(float *ax, float *ay, float *az)
{
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t data[6];

    i2c_master_write_read_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        &reg,
        1,
        data,
        6,
        pdMS_TO_TICKS(100)
    );

    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];

    *ax = raw_ax / 16384.0;
    *ay = raw_ay / 16384.0;
    *az = raw_az / 16384.0;
}


// ======================================================
// I2S INIT
// ======================================================

i2s_chan_handle_t rx_handle;

void i2s_microphone_init()
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_0,
        I2S_ROLE_MASTER
    );

    i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO
        ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK,
            .ws = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_SD,
        },
    };

    i2s_channel_init_std_mode(rx_handle, &std_cfg);
    i2s_channel_enable(rx_handle);
}


// ======================================================
// READ SOUND LEVEL
// ======================================================

float read_sound_level()
{
    int32_t samples[BUFFER_SIZE];
    size_t bytes_read;

    i2s_channel_read(
        rx_handle,
        samples,
        sizeof(samples),
        &bytes_read,
        pdMS_TO_TICKS(100)
    );

    int sample_count = bytes_read / sizeof(int32_t);

    double sum = 0;

    for (int i = 0; i < sample_count; i++)
    {
        int32_t sample = samples[i] >> 14;
        sum += sample * sample;
    }

    return sqrt(sum / sample_count);
}

// ======================================================
// OWB INIT
// ======================================================

OneWireBus * owb;
owb_gpio_driver_info driver_info;
OneWireBus_ROMCode device_rom_codes[MAX_DEVICES];
int num_devices = 0;

void owb_init(void)
{
    owb = owb_gpio_initialize(&driver_info,GPIO_DS18B20_0);
    owb_use_crc(owb, true);
    ESP_LOGI(TAG, "OneWire initialized");
}

bool owb_found_devices()
{
    num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;

    owb_search_first(owb, &search_state, &found);

    while (found && num_devices < MAX_DEVICES)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code,rom_code_s,sizeof(rom_code_s));
        ESP_LOGI(TAG, "  %d : %s",num_devices,rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb,&search_state,&found);
    }

    ESP_LOGI(TAG, "Found %d device%s", num_devices, num_devices == 1 ? "" : "s");

    return (num_devices > 0);
}

// ======================================================
// DS18B20 INIT 
// ======================================================

DS18B20_Info * ds18b20;

bool ds18b20_sensor_init()
{
    if (num_devices == 0)
    {
        ESP_LOGE(TAG, "No DS18B20 device found");
        return false;
    }

    ds18b20 = ds18b20_malloc();

    if (ds18b20 == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate DS18B20");
        return false;
    }

    ds18b20_init(ds18b20,owb,device_rom_codes[0]);
    ds18b20_use_crc(ds18b20,true);
    ds18b20_set_resolution(ds18b20,DS18B20_RESOLUTION_12_BIT);

    ESP_LOGI(TAG, "DS18B20 initialized");
    return true;
}

float read_temperature(void)
{
    ds18b20_convert_all(owb);
    ds18b20_wait_for_conversion(ds18b20);

    float temp = 0.0;

    DS18B20_ERROR err = ds18b20_read_temp(ds18b20,&temp);

    if (err == DS18B20_OK)
    {
        return temp;
    }

    ESP_LOGE(TAG,"Temperature read error");
    return NAN;
}

uint64_t get_timestamp_ms()
{
    return esp_timer_get_time() / 1000ULL;
}

float get_vibration_magnitude(float ax, float ay, float az)
{
    return sqrtf(ax * ax + ay * ay + az * az);
}

// ======================================================
// MAIN
// ======================================================

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    mpu6050_init();

    i2s_microphone_init();

    owb_init();
    if (!owb_found_devices())
    {
        ESP_LOGE(TAG, "No DS18B20 devices found on the bus");
        return;
    }
    if (!ds18b20_sensor_init())
    {
        ESP_LOGE(TAG, "Failed to initialize DS18B20 sensor");
        return;
    }

    float temp = read_temperature();
    uint64_t last_temp_update = get_timestamp_ms();

    printf("id,timestamp_ms,ax,ay,az,vib_mag,sound,temp,label\n");
    for (int i = 0; i < 10000; i++)  // logs 10000 samples (1000 seconds at 100ms intervals)
    {
        uint64_t timestamp_ms = get_timestamp_ms();
        float ax, ay, az;

        mpu6050_read_accel(&ax, &ay, &az);

        float sound = read_sound_level();

        // Update temperature only once per 10 seconds to reduce sensor read overhead
        if ((timestamp_ms - last_temp_update) >= 10000)
        {
            temp = read_temperature();
            last_temp_update = timestamp_ms;
        }

        // This used for test
        // ESP_LOGI(TAG,
        //          "TIMESTAMP_MS: %lu | ACCEL: X=%.2f Y=%.2f Z=%.2f | VIBRATION=%.2f | SOUND=%.2f | TEMP=%.2f | LABEL=%s",
        //          get_timestamp_ms(),
        //          ax, ay, az, get_vibration_magnitude(ax, ay, az),
        //          sound,
        //          temp,
        //          LABEL);
        
        // CSV data logging format
        printf("%d,%llu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n", 
                i,
                get_timestamp_ms(), 
                ax, ay, az, get_vibration_magnitude(ax, ay, az), 
                sound, 
                temp, 
                LABEL);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Data collection complete");
}