/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"
#include "sdcard.h"

static const char *TAG = "RTC_BENCHMARK";

// Wi-Fi credentials
#define WIFI_SSID      "TP-LINK_7CF2"
#define WIFI_PASS      "gtUp5xWL"
#define MAXIMUM_RETRY  5

// Experiment parameters
#define SAMPLE_INTERVAL_MINUTES 10
#define EXPERIMENT_DURATION_HOURS 10

static int s_retry_num = 0;
static SemaphoreHandle_t s_time_sync_sem;

void initialize_sntp(void);

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            ESP_LOGE(TAG, "Failed to connect to the AP");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        // Start SNTP synchronization
        initialize_sntp();
    }
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized");
    // Set timezone to Central European Time
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    
    // Notify the main task that time is synchronized
    xSemaphoreGive(s_time_sync_sem);
}

void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

void wifi_init_sta(void)
{
    s_time_sync_sem = xSemaphoreCreateBinary();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished. Waiting for time synchronization.");
    
    // Wait for time synchronization with a timeout
    if (xSemaphoreTake(s_time_sync_sem, pdMS_TO_TICKS(60000)) == pdTRUE) { // 1 minute timeout
        ESP_LOGI(TAG, "Initial time synchronization complete.");
    } else {
        ESP_LOGE(TAG, "Failed to synchronize time within 60 seconds. Aborting experiment.");
        // You might want to handle this error more gracefully, e.g. by restarting
    }
}


void app_main(void)
{
    const char filename[32];
    ESP_LOGI(TAG, "------------------------------INITIALIZATION------------------------------");
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // This will initialize Wi-Fi and wait for the first NTP sync
    wifi_init_sta();

    ESP_LOGI(TAG, "Initializing SD card...");
    sdcardinitialize();
    int index = sd_write_index(MOUNT_POINT"/index.txt");
    sprintf(filename, MOUNT_POINT"/data%d.csv", index);
    sd_write_file_header(filename);
/*
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    const char mount_point[] = "/sdcard";
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card (%s).", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "SD card mounted successfully.");
    sdmmc_card_print_info(stdout, card);
*/
    ESP_LOGI(TAG, "--------------------------------------------------------------------------");

    ESP_LOGI(TAG, "--------------------------------------RTC Benchmark--------------------------------------");

    // Open log file
    //
   /*
    const char* log_file_path = "/sdcard/drift_log.csv";
    ESP_LOGI(TAG, "Opening log file: %s", log_file_path);
    FILE* f = fopen(log_file_path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "NTP Time,RTC Time,Offset (s)\n");
    fclose(f);
*/
    // Record start times
    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
    int64_t rtc_start_micros = esp_timer_get_time();

    char time_str_buf[64];
    strftime(time_str_buf, sizeof(time_str_buf), "%c", localtime(&tv_start.tv_sec));
    ESP_LOGI(TAG, "Experiment started at: %s", time_str_buf);

    const int num_samples = (EXPERIMENT_DURATION_HOURS * 60) / SAMPLE_INTERVAL_MINUTES;
    const int delay_ms = SAMPLE_INTERVAL_MINUTES * 60 * 1000;

    for (int i = 0; i < num_samples; i++) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        // Get current times
        struct timeval tv_current;
        gettimeofday(&tv_current, NULL);
        int64_t rtc_current_micros = esp_timer_get_time();

        // Calculate elapsed times in seconds
        double elapsed_ntp_s = (double)(tv_current.tv_sec - tv_start.tv_sec) + 
                               (double)(tv_current.tv_usec - tv_start.tv_usec) / 1000000.0;
        double elapsed_rtc_s = (double)(rtc_current_micros - rtc_start_micros) / 1000000.0;

        // Calculate offset
        double offset_s = elapsed_rtc_s - elapsed_ntp_s;

        // Calculate the current time according to the RTC
        struct timeval tv_rtc_current;
        tv_rtc_current.tv_sec = tv_start.tv_sec + (time_t)elapsed_rtc_s;
        tv_rtc_current.tv_usec = tv_start.tv_usec + (long)((elapsed_rtc_s - (long)elapsed_rtc_s) * 1000000.0);
        if (tv_rtc_current.tv_usec >= 1000000) {
            tv_rtc_current.tv_sec++;
            tv_rtc_current.tv_usec -= 1000000;
        }

        // Format time strings
        char ntp_time_str[64];
        char rtc_time_str[64];
        strftime(ntp_time_str, sizeof(ntp_time_str), "%Y-%m-%d %H:%M:%S", localtime(&tv_current.tv_sec));
        strftime(rtc_time_str, sizeof(rtc_time_str), "%Y-%m-%d %H:%M:%S", localtime(&tv_rtc_current.tv_sec));

        ESP_LOGI(TAG, "Sample %d/%d: NTP: %s, RTC: %s, Offset: %.6f s", 
                 i + 1, num_samples, ntp_time_str, rtc_time_str, offset_s);

        // Log to SD card
        sd__write_file(filename,ntp_time_str, rtc_time_str, offset_s);
        /*
        f = fopen(log_file_path, "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for appending");
            continue; // Skip this sample if file can't be opened
        }
        fprintf(f, "%s,%s,%.6f\n", ntp_time_str, rtc_time_str, offset_s);
        fclose(f);
        */
    }

    ESP_LOGI(TAG, "Experiment finished.");

}
