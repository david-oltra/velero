#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_http_client.h>
#include <esp_wifi.h>
#include <esp_https_ota.h>
#include <stdio.h>
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <hmc5883l.h>
#include <cmath>

#include <version.h>
#include <secret.h>

#define ENABLE_OTA
#ifdef ENABLE_OTA
extern "C" void wifi_init_sta();
#endif
#ifndef WIFI_PWD
#error Missing WIFI_PWD Environment variable
#endif
#ifndef WIFI_SSID
#error Missing WIFI_SSID Environment variable
#endif

extern "C" void app_main();

static char TAG[] = "MAIN";

static TaskHandle_t core_A = NULL;
static TaskHandle_t core_B = NULL;

HMC5883L compass;

void EnableWifi()
{
    #ifdef ENABLE_OTA
        wifi_init_sta();
    #endif
}

void updateOTA()
{
    static char TAG_OTA[] = "OTA";
    ESP_LOGI(TAG_OTA, "Starting OTA task");
    esp_http_client_config_t config = {
        .url = "http://192.168.0.16:8000/firmware.bin?",
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    ESP_LOGI(TAG_OTA, "Attempting to download update from %s", config.url);
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG_OTA, "OTA Succeed, Rebooting...");
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG_OTA, "Firmware upgrade failed");
    }
}

void checkVersionFW()
{
    static char TAG_OTA[] = "OTA";
    char newVersionFW[50];
    memset(newVersionFW, 0, 50);

    esp_http_client_config_t configFW = {
        .url = "http://192.168.0.16:8000/VERSION.html",
        .user_data = newVersionFW,        // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&configFW);
    esp_http_client_open(client,0);
    esp_http_client_fetch_headers(client);
    esp_http_client_read(client, newVersionFW, esp_http_client_get_content_length(client));

    ESP_LOGE(TAG_OTA,"Received html: %s", newVersionFW);
    uint64_t value;
    value = atoll(newVersionFW);
    ESP_LOGE(TAG_OTA,"Received html: %llu", value);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);  

    if (value > actualVersion)
    {   
        ESP_LOGE(TAG_OTA, "NEW VERSION!!!");
        updateOTA();
    }
}

void init_gpios()
{
    gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << GPIO_NUM_15);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(GPIO_NUM_15,0);
}

void init_pwm()
{
    ledc_timer_config_t ledc_timer_A;
    ledc_timer_A.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer_A.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer_A.timer_num        = LEDC_TIMER_0;
    ledc_timer_A.freq_hz          = 50;
    ledc_timer_A.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_A));

    ledc_channel_config_t ledc_channel_A;
    ledc_channel_A.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel_A.channel        = LEDC_CHANNEL_0;
    ledc_channel_A.timer_sel      = LEDC_TIMER_0;
    ledc_channel_A.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel_A.gpio_num       = GPIO_NUM_5;
    ledc_channel_A.duty           = 0; // 0 -> 2^13 -1 = 8192 - 1
    ledc_channel_A.hpoint         = 0;
    ledc_channel_A.flags.output_invert  = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_A));

    ledc_timer_config_t ledc_timer_B;
    ledc_timer_B.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer_B.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer_B.timer_num        = LEDC_TIMER_1;
    ledc_timer_B.freq_hz          = 50;
    ledc_timer_B.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_B));

    ledc_channel_config_t ledc_channel_B;
    ledc_channel_B.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel_B.channel        = LEDC_CHANNEL_1;
    ledc_channel_B.timer_sel      = LEDC_TIMER_1;
    ledc_channel_B.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel_B.gpio_num       = GPIO_NUM_4;
    ledc_channel_B.duty           = 0;
    ledc_channel_B.hpoint         = 0;
    ledc_channel_B.flags.output_invert  = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_B));
}

void init_i2c()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.scl_io_num = GPIO_NUM_22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;

    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void init_compass()
{
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_1);
    compass.setOffset(71, -486);
}

uint16_t wind_degrees()
{
    uint8_t rx_data[2];
    uint8_t device_addr = 0x36;
    uint8_t data_addr = 0x0E;   //0x0E ANGLE(11:8)  0x0F ANGLE(7:0)
    uint16_t data_degrees = 0;

    i2c_master_write_read_device(I2C_NUM_0,
                                 device_addr,
                                 &data_addr,
                                 1,
                                 rx_data,
                                 2,
                                 pdMS_TO_TICKS(10));

    data_degrees = 360 * (rx_data[0] << 8 | rx_data[1]) / 4096;
    printf("grados: %u\n", data_degrees);

    return data_degrees;
}

//ESP_LOG_BUFFER_HEX(TAG, rx_data, len_data);

float distance(float lat_from, float long_from, float lat_to, float long_to)
{
    float lat_from_rad = lat_from * 3.14159265 / 180;
    float long_from_rad = long_from * 3.14159265 / 180;
    float lat_to_rad = lat_to * 3.14159265 / 180;
    float long_to_rad = long_to * 3.14159265 /180;

    int16_t earth_radius = 6371;

    float distance = 2 * earth_radius * asin( sqrt( pow(sin((lat_to_rad - lat_from_rad)/2),2) + cos(lat_to_rad) * cos(lat_from_rad) * pow(sin((long_to_rad - long_from_rad)/2),2)));

    return distance;
}

float orientation(float lat_from, float long_from, float lat_to, float long_to)
{
    int16_t earth_polar_radius = 6357;

    float dif_lat = (lat_to - lat_from) * ((2 * 3.14159265 * earth_polar_radius)/360);

    float degrees = acos( dif_lat / distance(lat_from, long_from, lat_to, long_to)) * 180 / 3.14159265;

    if(long_to < long_from) degrees = degrees * -1;

    return degrees;
}

void coreAThread(void *arg)
{
    static char TAG_CORE_A[] = "CORE_A";
    ESP_LOGE(TAG_CORE_A, "Starting CORE A");  
    while(1)
    {
        uint8_t deg = ((wind_degrees()*115/360))+20;
        /**
         * 135-20=115
         */
        printf("wind_deg = %u\t",deg);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, deg));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, deg));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void coreBThread(void *arg)
{
    static char TAG_CORE_B[] = "CORE_B";
    ESP_LOGE(TAG_CORE_B, "Starting CORE B");  
    while(1)
    {
        printf("orientation degrees N: %.2f\n", compass.getDegrees());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    ESP_LOGE(TAG, "Starting software");
    if (esp_wifi_connect() == ESP_OK)
    {
        checkVersionFW();
    }
    init_gpios();
    init_pwm();
    init_i2c();
    init_compass();
    
    xTaskCreatePinnedToCore(coreBThread, "core_B", 4096, NULL, 9, &core_B, 1);    
    xTaskCreatePinnedToCore(coreAThread, "core_A", 4096, NULL, 10, &core_A, 0);
}


