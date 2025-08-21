#include <stdio.h>
#include <string.h>
#include <time.h>
#include "driver/mcpwm_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "gps_redo.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
 #include "sdcard.h"
 #include "math.h"


// experiment parameters
#define EXPERIMENT_DURATION_HOURS 24
#define INITIAL_SYNC_TIMEOUT_MINUTES 5
#define PPS_GPIO_PIN 4
#define CAPTURE_WINDOW_SIZE 50

// hardware capture handles
static mcpwm_cap_timer_handle_t cap_timer = NULL;
static mcpwm_cap_channel_handle_t cap_channel = NULL;

static const char *TAG = "GPS_RTC_BENCHMARK";

typedef struct {
    uint32_t cap_value;
    uint64_t sys_time_us;
} pps_hw_capture;

static volatile pps_hw_capture captures[CAPTURE_WINDOW_SIZE];
static volatile int capture_index = 0;
static volatile uint64_t total_captures = 0;
static volatile uint64_t last_callback_time = 0;  // last ISR time (us)

static bool IRAM_ATTR pps_hw_capture_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data){
    // hardware-captured timestamp
    uint32_t hw_timestamp = edata->cap_value;

    // system timestamp for comparison
    uint64_t sys_timestamp = esp_timer_get_time();

    // store in circular buffer
    int idx = capture_index % CAPTURE_WINDOW_SIZE;
    captures[idx].cap_value = hw_timestamp;
    captures[idx].sys_time_us = sys_timestamp;

    capture_index++;
    total_captures++;
    last_callback_time = sys_timestamp;

    // feed GPS jitter/PPS accounting with this hardware-captured pulse
    gps_record_pps_pulse(sys_timestamp);

    return false;
}

// initialize MCPWM hardware capture on GPIO 4
esp_err_t init_hardware_pps_capture(void) {
    ESP_LOGI(TAG, "Initializing MCPWM hardware capture on GPIO %d", PPS_GPIO_PIN);

    mcpwm_capture_timer_config_t cap_timer_config = {
    .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    .group_id = 0,
    };
    esp_err_t ret = mcpwm_new_capture_timer(&cap_timer_config, &cap_timer);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a capture timer: %s", esp_err_to_name(ret));
        return ret;
    }

    mcpwm_capture_channel_config_t cap_chan_config = {
    .gpio_num= PPS_GPIO_PIN,
    .prescale = 1,
    .flags.neg_edge = false,
    .flags.pos_edge = true,
    .flags.pull_up = true,
    };

    ret = mcpwm_new_capture_channel(cap_timer, &cap_chan_config, &cap_channel);
    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to create a capture channel: %s", esp_err_to_name(ret));
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    mcpwm_capture_event_callbacks_t cbs = {
    .on_cap = pps_hw_capture_callback,
    };

    ret = mcpwm_capture_channel_register_event_callbacks(cap_channel, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register capture callbacks: %s", esp_err_to_name(ret));
        mcpwm_del_capture_channel(cap_channel);
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    // enable capture channel
    ret = mcpwm_capture_channel_enable(cap_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable capture channel: %s", esp_err_to_name(ret));
        mcpwm_del_capture_channel(cap_channel);
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    // enable and start capture timer
    ret = mcpwm_capture_timer_enable(cap_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable capture timer: %s", esp_err_to_name(ret));
        mcpwm_capture_channel_disable(cap_channel);
        mcpwm_del_capture_channel(cap_channel);
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    ret = mcpwm_capture_timer_start(cap_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start capture timer: %s", esp_err_to_name(ret));
        mcpwm_capture_timer_disable(cap_timer);
        mcpwm_capture_channel_disable(cap_channel);
        mcpwm_del_capture_channel(cap_channel);
        mcpwm_del_capture_timer(cap_timer);
        return ret;
    }

    ESP_LOGI(TAG, "MCPWM hardware capture initialized");

    return ESP_OK;
}

void analyze_hardware_captures(void) {
    if (total_captures < 3) {
        ESP_LOGI(TAG, "Waiting for more captures... (%llu so far)", total_captures);
        return;
    }

    // determine how many valid intervals we can calculate
    int available_samples = (total_captures < CAPTURE_WINDOW_SIZE) ? total_captures : CAPTURE_WINDOW_SIZE;
    int num_intervals = available_samples - 1;  // we need N-1 intervals for N samples
    
    if (num_intervals < 2) {
        ESP_LOGI(TAG, "Not enough intervals for analysis (need at least 2, have %d)", num_intervals);
        return;
    }

    double hw_intervals_ns[CAPTURE_WINDOW_SIZE];
    
    // calculate hardware intervals in nanoseconds
    for (int i = 0; i < num_intervals; i++) {
        // for circular buffer: get the indices correctly
        int curr_idx, prev_idx;
        
        if (total_captures < CAPTURE_WINDOW_SIZE) {
            // buffer not yet wrapped, simple indexing
            curr_idx = i + 1;
            prev_idx = i;
        } else {
            // buffer has wrapped, use circular indexing
            curr_idx = (capture_index - num_intervals + i) % CAPTURE_WINDOW_SIZE;
            prev_idx = (capture_index - num_intervals + i - 1) % CAPTURE_WINDOW_SIZE;
        }

        uint32_t hw_interval_ticks = captures[curr_idx].cap_value - captures[prev_idx].cap_value;
        hw_intervals_ns[i] = hw_interval_ticks * 12.5;  // convert to nanoseconds (80MHz = 12.5ns per tick)
        
        // debug: log first few intervals
        if (i < 3) {
            ESP_LOGI(TAG, "Interval %d: %lu ticks = %.1f ns (%.6f ms)", 
                     i, hw_interval_ticks, hw_intervals_ns[i], hw_intervals_ns[i] / 1000000.0);
        }
    }

    // calculate statistics
    double mean_ns = 0.0;
    for (int i = 0; i < num_intervals; i++) {
        mean_ns += hw_intervals_ns[i];
    }
    mean_ns /= num_intervals;

    double variance = 0.0;
    for (int i = 0; i < num_intervals; i++) {
        double diff = hw_intervals_ns[i] - mean_ns;
        variance += diff * diff;
    }
    variance /= (num_intervals - 1);
    double jitter_ns = sqrt(variance);

    ESP_LOGI(TAG, "Hardware capture: N=%d, mean=%.6f ms, jitter=%.1f ns (caps=%llu)",
             num_intervals, mean_ns / 1000000.0, jitter_ns, total_captures);
}

void app_main(void)
{
    ESP_LOGI(TAG, "GPS RTC Benchmark init");
    
    // initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // initialize sd card
    ESP_LOGI(TAG, "Initializing SD card");
    sdcardinitialize();

    ESP_LOGI(TAG, "Bringing up MCPWM hardware capture");
    ret = init_hardware_pps_capture();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize hardware capture!");
        return;
    }

    // create log file with headers
    int index = sd_write_index(MOUNT_POINT"/index.txt");
    char filename[64];
    sprintf(filename, MOUNT_POINT"/data%d.csv", index);
    sd_write_file_header(filename);

    // initialize gps and pps
    ESP_LOGI(TAG, "Initializing GPS");
    gps_initialize();
    
    ESP_LOGI(TAG, "GPS RTC Benchmark starting");
    
    // start GPS and benchmark tasks
    start_gps_benchmark_tasks(filename);
    
    // wait for initial GPS time synchronization
    ESP_LOGI(TAG, "Waiting for GPS time synchronization (timeout: %d minutes)...", INITIAL_SYNC_TIMEOUT_MINUTES);
    int sync_timeout_seconds = INITIAL_SYNC_TIMEOUT_MINUTES * 60;
    int check_interval_ms = 1000; // check every second
    int elapsed_seconds = 0;
    
    // wait for the synchronization of rtc with the local time fetched through NMEA
    while (elapsed_seconds < sync_timeout_seconds) {
        if (time_synchronized) {
            ESP_LOGI(TAG, "GPS time synchronization achieved after %d seconds", elapsed_seconds);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(check_interval_ms));
        elapsed_seconds++;
    }
    
    if (!time_synchronized) {
        ESP_LOGE(TAG, "Failed to synchronize with GPS within %d minutes. Continuing anyway...", INITIAL_SYNC_TIMEOUT_MINUTES);
    }
    
    // record experiment start time
    uint64_t experiment_start_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Run for %d hours; GPS on UART2 (TX:17 RX:16 PPS:4). Logs every 10 min -> %s", EXPERIMENT_DURATION_HOURS, filename);
    
    // main monitoring loop - the periodic comparison is handled by rtc_comparison_task
    uint64_t experiment_duration_us = (uint64_t)EXPERIMENT_DURATION_HOURS * 60 * 60 * 1000000;
    uint64_t last_status_time = 0;
    // throttled status logging
    
    while (1) {
        uint64_t current_time = esp_timer_get_time();
        uint64_t elapsed_time = current_time - experiment_start_time;
        
        if (elapsed_time >= experiment_duration_us) {
            ESP_LOGI(TAG, "Experiment duration reached. Finishing benchmark.");
            break;
        }
        
        // log status every hour
        if (current_time - last_status_time >= 60ULL * 60ULL * 1000000ULL) { // 1 hour in microseconds
            uint64_t hours_elapsed = elapsed_time / (60ULL * 60ULL * 1000000ULL);
            uint64_t hours_remaining = EXPERIMENT_DURATION_HOURS - hours_elapsed;
            
            ESP_LOGI(TAG, "Benchmark Status - %llu/%d hours completed, %llu hours remaining, PPS count: %llu", 
                     hours_elapsed, EXPERIMENT_DURATION_HOURS, hours_remaining, pps_count);
            
            // analyze hardware capture performance
            analyze_hardware_captures();
            
            last_status_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // check every 5 seconds
    }
    
    ESP_LOGI(TAG, "GPS RTC Benchmark completed successfully!");
    ESP_LOGI(TAG, "Data logged to: %s", filename);
    
    // final summary
    ESP_LOGI(TAG, "Final PPS count: %llu pulses over %d hours", pps_count, EXPERIMENT_DURATION_HOURS);
    ESP_LOGI(TAG, "Average PPS rate: %.2f Hz", (float)pps_count / (EXPERIMENT_DURATION_HOURS * 3600.0));
}
