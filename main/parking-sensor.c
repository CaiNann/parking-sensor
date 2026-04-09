#include "freertos/FreeRTOS.h"
#include "wifi.h"
#include "passwords.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_http_client.h"

const static char *TAG = "example";

#define HC_SR04_TRIG_GPIO  25
#define HC_SR04_ECHO_GPIO  26

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    //calculate the interval in the ISR,
    //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

/**
 * @brief generate single pulse on Trig pin to start a new sample
 */
static void gen_trig_output(void)
{
    gpio_set_level(HC_SR04_TRIG_GPIO, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_TRIG_GPIO, 0); // set low
}

static esp_err_t connect_to_wifi(void) {
    ESP_LOGI(TAG, "Initializing Wi-Fi");
       
    esp_err_t init_ret = app_wifi_init();
    if (init_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Wi-Fi");
        return -1;
    }
    
    int retry_count = 0;
    
    ESP_LOGI(TAG, "Attempting to connect to Wi-Fi");
    while (retry_count < 3) {
        esp_err_t ret = app_wifi_connect(WIFI_SSID, WIFI_PASSWORD);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect to Wi-Fi, retrying...");
            retry_count++;
        }
        break;
    }
    
    if (retry_count == 3) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi after 3 retries");
        esp_err_t ret = app_wifi_deinit();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to deinitialize Wi-Fi");
        }
        return -1;
    }
    
    return ESP_OK;
}

void http_post_task(void *pvParameters) {
    float value = *(float *)pvParameters;
    
    char payload[64];
    
    snprintf(payload, sizeof(payload), "{\"distance\": %.2f}", value);
    
    esp_http_client_config_t config = {
        .url = SERVER_ADDR,
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_post_field(client, (const char *)payload, strlen(payload));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI("HTTP", "Status = %d", esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE("HTTP", "Request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    vTaskDelete(NULL); 
}

void app_main(void)
{
    ESP_LOGI(TAG, "Connecting to wifi...");
    ESP_ERROR_CHECK(connect_to_wifi());
    
    ESP_LOGI(TAG, "Starting parking sensor");
    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));
    ESP_ERROR_CHECK(gpio_set_pull_mode(HC_SR04_ECHO_GPIO, GPIO_PULLUP_ONLY));

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    uint32_t tof_ticks;
    while (1) {
        // trigger the sensor to start a new sample
        gen_trig_output();
        // wait for echo done signal
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us > 35000) {
                // out of range
                continue;
            }
            // convert the pulse width into measure distance
            float distance = (float) pulse_width_us / 58;
            ESP_LOGI(TAG, "Measured distance: %.2fcm", distance);
            xTaskCreate(http_post_task, "http_post_task", 8192, &distance, 5, NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}