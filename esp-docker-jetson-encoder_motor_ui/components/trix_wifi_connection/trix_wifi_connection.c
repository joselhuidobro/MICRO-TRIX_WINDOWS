#include "trix_wifi_connection.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <string.h>

/* ───── Configuración ───── */
#define WIFI_SSID          "IZZI-9854"
#define WIFI_PASSWORD      "YkcX96cGJsGEGectbx"
#define LED_BUILTIN        GPIO_NUM_2
#define MAX_RETRY               5
#define LED_BLINK_MS_FAST     100
#define LED_BLINK_MS_SLOW     500

/* ───── Bits de evento ───── */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char* TAG = "TRIX_WIFI";
static EventGroupHandle_t wifi_event_group;
static volatile bool wifi_connected = false;
static volatile bool blink_fast = true;

/* ───── Prototipos ───── */
static void wifi_event_handler(void*, esp_event_base_t, int32_t, void*);
static void wifi_led_task(void*);

/* ───── Implementación ───── */
void trix_wifi_init(void)
{
    /* LED */
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);

    /* NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Netif + eventos */
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* Wi-Fi init */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                            &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {};
    strcpy((char*)wifi_cfg.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_cfg.sta.password, WIFI_PASSWORD);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Tarea LED */
    xTaskCreatePinnedToCore(wifi_led_task, "wifi_led_task", 2048,
                            NULL, 5, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Wi-Fi inicializado; intentando conexión ...");
}

bool trix_wifi_is_connected(void)
{
    return wifi_connected;
}

/* ───── Event handler ───── */
static void wifi_event_handler(void* arg,
                               esp_event_base_t base,
                               int32_t id,
                               void* data)
{
    static int retry_count = 0;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        blink_fast = true;

        if (retry_count < MAX_RETRY) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGW(TAG, "Reintentando (%d/%d) ...", retry_count, MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "No se pudo conectar al AP");
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "Conectado - IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count   = 0;
        wifi_connected = true;
        blink_fast    = false;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ───── LED task ───── */
static void wifi_led_task(void* arg)
{
    while (true) {
        uint32_t delay_ms = blink_fast ? LED_BLINK_MS_FAST : LED_BLINK_MS_SLOW;

        if (wifi_connected) {
            gpio_set_level(LED_BUILTIN, 1);          // fijo encendido
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        } else {
            gpio_set_level(LED_BUILTIN, 1);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            gpio_set_level(LED_BUILTIN, 0);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

