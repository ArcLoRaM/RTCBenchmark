#include <stdio.h>
#include <string.h>
#include <time.h>
#include "driver/mcpwm_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "gps_redo.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
 #include "sdcard.h"
 #include "math.h"
 #include <stdlib.h>
#include "ds3231.h"
#include "driver/i2c.h"

static const char *TAG = "RTC_BENCHMARK";

// ds3231 variables
static DS3231_Info g_ds3231;

// gpio pins
#define PPS_GPIO_PIN 4
#define SQW_GPIO_PIN 2

// experiment parameters
#define EXPERIMENT_DURATION_HOURS 24
#define INITIAL_SYNC_TIMEOUT_MINUTES 5
#define CAPTURE_WINDOW_SIZE 50

// hardware capture handles
static mcpwm_cap_timer_handle_t cap_timer = NULL;
static mcpwm_cap_channel_handle_t cap_channel = NULL;
static mcpwm_cap_channel_handle_t cap_channel_sqw = NULL;

// PPS counter is updated in gps_redo.c; use it to align snapshot to the next PPS edge
extern volatile uint64_t pps_count;
// Access RTC slow clock time (us)
extern uint64_t esp_rtc_get_time_us(void);

typedef struct {
    uint32_t cap_value;
    uint64_t sys_time_us;
} pps_hw_capture;

static volatile pps_hw_capture captures[CAPTURE_WINDOW_SIZE];
static volatile int capture_index = 0;
static volatile uint64_t total_captures = 0;
static volatile uint64_t last_callback_time = 0;  // last ISR time (us)

// protect capture ring buffer against concurrent reads in analyzer
static portMUX_TYPE cap_mux = portMUX_INITIALIZER_UNLOCKED;

// SQW capture state
static volatile uint64_t last_sqw_time_us = 0;
static volatile uint64_t total_sqw_captures = 0;
static volatile int64_t last_pps_sqw_phase_us = 0; // signed minimal phase (PPS - SQW) in [-500000, 500000]
static volatile uint32_t last_sqw_cap_ticks = 0;     // last SQW capture timer value (80 MHz ticks)
static volatile bool has_sqw_cap = false;

// pps boundary captured slow-clock and count (for pps synchronous logging)
static volatile uint64_t last_rtc32k_at_pps = 0;      // esp_rtc_get_time_us() captured in PPS ISR
static volatile uint64_t last_pps_count_captured = 0; // pps_count value at that PPS

// logger task handle for direct notification (every 600th PPS)
static TaskHandle_t s_pps_logger_task = NULL;

// expose the captured values for the logger in gps_redo.c
uint64_t get_last_rtc32k_at_pps(void) { return last_rtc32k_at_pps; }
uint64_t get_last_pps_count_captured(void) { return last_pps_count_captured; }

// expose latest PPS–SQW phase (in microseconds) for logging elsewhere
int64_t get_last_pps_sqw_phase_us(void) {
    return last_pps_sqw_phase_us;
}

// provide the last SQW edge timestamp in microseconds
uint64_t get_last_sqw_edge_time_us(void) {
    return last_sqw_time_us;
}

static bool IRAM_ATTR pps_hw_capture_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data){
    // hardware captured timestamp
    uint32_t hw_timestamp = edata->cap_value;

    // system timestamp for comparison
    uint64_t sys_timestamp = esp_timer_get_time();

    // store in circular buffer
    portENTER_CRITICAL_ISR(&cap_mux);
    int idx = capture_index % CAPTURE_WINDOW_SIZE;
    captures[idx].cap_value = hw_timestamp;
    captures[idx].sys_time_us = sys_timestamp;
    capture_index++;
    total_captures++;
    last_callback_time = sys_timestamp;
    portEXIT_CRITICAL_ISR(&cap_mux);

    // feed GPS jitter/PPS accounting with this hardware-captured pulse
    gps_record_pps_pulse(sys_timestamp);

    // capture slow-clock at PPS boundary and the corresponding count
    last_pps_count_captured = pps_count;
    last_rtc32k_at_pps = esp_rtc_get_time_us();

    // update PPS–SQW phase using 80 MHz capture ticks for sub-µs resolution
    if (has_sqw_cap) {
        // compute signed delta ticks and wrap
        int32_t delta_ticks = (int32_t)(hw_timestamp - last_sqw_cap_ticks);
        const int32_t one_s_ticks = 80000000;  // 80 MHz
        const int32_t half_s_ticks = one_s_ticks / 2; // 40,000,000
        if (delta_ticks > half_s_ticks) delta_ticks -= one_s_ticks;
        else if (delta_ticks < -half_s_ticks) delta_ticks += one_s_ticks;
        // convert to microseconds with rounding: us = ticks / 80
        if (delta_ticks >= 0) last_pps_sqw_phase_us = (delta_ticks + 40) / 80;
        else last_pps_sqw_phase_us = (delta_ticks - 40) / 80;
    }

    // notify logger every 60th PPS after sync (skip 0)
    const uint64_t PERIOD_PPS = 60ULL; // 60 seconds
    if (s_pps_logger_task && last_pps_count_captured != 0 && (last_pps_count_captured % PERIOD_PPS) == 0) {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(s_pps_logger_task, &hpw);
        if (hpw) portYIELD_FROM_ISR();
    }

    return false;
}

// sqw capture callback: record timestamp from the ds3231 and count
static bool IRAM_ATTR sqw_hw_capture_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data){
    (void)cap_chan; (void)user_data;
    uint64_t sys_timestamp = esp_timer_get_time();
    last_sqw_time_us = sys_timestamp;
    last_sqw_cap_ticks = edata->cap_value;
    has_sqw_cap = true;
    total_sqw_captures++;
    return false;
}

// ------------------initialize MCPWM hardware capture ------------------------
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

    ESP_LOGI(TAG, "MCPWM hardware capture initialized on GPIO %d", PPS_GPIO_PIN);

    // ------------------ initialize SQW capture -----------------------
    mcpwm_capture_channel_config_t sqw_chan_config = {
        .gpio_num = SQW_GPIO_PIN,
        .prescale = 1,
        .flags.neg_edge = false,
        .flags.pos_edge = true,
        .flags.pull_up = true,
    };

    esp_err_t ret2 = mcpwm_new_capture_channel(cap_timer, &sqw_chan_config, &cap_channel_sqw);
    if (ret2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create SQW capture channel on GPIO %d: %s", SQW_GPIO_PIN, esp_err_to_name(ret2));
        return ESP_OK;
    }

    mcpwm_capture_event_callbacks_t sqw_cbs = { .on_cap = sqw_hw_capture_callback };
    ret2 = mcpwm_capture_channel_register_event_callbacks(cap_channel_sqw, &sqw_cbs, NULL);
    if (ret2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register SQW capture callback: %s", esp_err_to_name(ret2));
        // continue without SQW
        return ESP_OK;
    }

    ret2 = mcpwm_capture_channel_enable(cap_channel_sqw);
    if (ret2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable SQW capture channel: %s", esp_err_to_name(ret2));
        return ESP_OK;
    }

    ESP_LOGI(TAG, "SQW capture initialized on GPIO %d", SQW_GPIO_PIN);
    return ESP_OK;
// ---------------------------------------------------------------------------
}

void analyze_hardware_captures(void) {
    // take snapshot of ring buffer indices and the last samples
    uint32_t local_ci;
    uint64_t local_total;
    pps_hw_capture local_buf[CAPTURE_WINDOW_SIZE];

    portENTER_CRITICAL(&cap_mux);
    local_ci = (uint32_t)capture_index;
    local_total = total_captures;
    // determine how many samples are available and copy them in chronological order
    int available_samples = (local_total < CAPTURE_WINDOW_SIZE) ? (int)local_total : CAPTURE_WINDOW_SIZE;
    int start = (int)((local_ci - available_samples) % CAPTURE_WINDOW_SIZE);
    if (start < 0) start += CAPTURE_WINDOW_SIZE; // guard for negative modulo on some compilers
    for (int i = 0; i < available_samples; i++) {
        int src = (start + i) % CAPTURE_WINDOW_SIZE;
        local_buf[i] = (pps_hw_capture) { .cap_value = captures[src].cap_value, .sys_time_us = captures[src].sys_time_us };
    }
    portEXIT_CRITICAL(&cap_mux);

    if (available_samples < 3) {
        ESP_LOGI(TAG, "Waiting for more captures... (%llu so far)", total_captures);
        return;
    }

    // determine how many valid intervals we can calculate
    int num_intervals = available_samples - 1;  // we need N-1 intervals for N samples
    
    if (num_intervals < 2) {
        ESP_LOGI(TAG, "Not enough intervals for analysis (need at least 2, have %d)", num_intervals);
        return;
    }

    double hw_intervals_ns[CAPTURE_WINDOW_SIZE];
    
    // calculate hardware intervals 
    for (int i = 0; i < num_intervals; i++) {
        // contiguous intervals from the local snapshot
        uint32_t hw_interval_ticks = (uint32_t)(local_buf[i + 1].cap_value - local_buf[i].cap_value);
        hw_intervals_ns[i] = hw_interval_ticks * 12.5;  // convert to nanoseconds (80MHz = 12.5ns per tick)
        
        // debug: log first few intervals
        if (i < 3) {
            ESP_LOGI(TAG, "Interval %d: %lu ticks = %.1f ns (%.6f ms)",
                     i, (unsigned long)hw_interval_ticks, hw_intervals_ns[i], hw_intervals_ns[i] / 1000000.0);
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
             num_intervals, mean_ns / 1000000.0, jitter_ns, local_total);
}

// logger task blocks until ISR notifications (exact 600-PPS cadence)
static void pps_boundary_logger_task(void* arg) {
    char* fn = (char*)arg;
    while (!time_synchronized) { vTaskDelay(pdMS_TO_TICKS(50)); }
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        log_time_comparison_at_pps(fn);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "GPS RTC Benchmark init");
    // force UTC for all time conversions (avoid DST/local offsets)
    setenv("TZ", "UTC0", 1);
    tzset();
    
    // initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // initialize ds3231
    ESP_LOGI(TAG, "Initializing DS3231 RTC");
    ds3231_init_info(&g_ds3231, I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22,1000);
    // give bus/device a moment to settle
    vTaskDelay(pdMS_TO_TICKS(10));
    struct tm t;
    esp_err_t dsr = ds3231_get_time(&g_ds3231, &t);
    if (dsr == ESP_OK) {
        ESP_LOGI(TAG, "DS3231 present (UTC %04d-%02d-%02d %02d:%02d:%02d)", t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        ESP_LOGW(TAG, "DS3231 not responding on I2C0 (err=%d)", dsr);
    }

    esp_err_t sqw = ds3231_enable_sqw_1hz(&g_ds3231, true);
    if (sqw == ESP_OK) {
        ESP_LOGI(TAG, "DS3231 SQW 1 Hz enabled");
    } else {
        ESP_LOGW(TAG, "Failed to enable DS3231 SQW 1 Hz (err=%d)", sqw);
    }

    // make DS3231 available to GPS code for PPS-aligned updates
    gps_register_ds3231(&g_ds3231);


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

    // start pps boundary logger
    char* filename_copy2 = malloc(strlen(filename) + 1);
    if (filename_copy2) {
        strcpy(filename_copy2, filename);
    xTaskCreate(pps_boundary_logger_task, "pps_boundary_logger", 4096, filename_copy2, 3, &s_pps_logger_task);
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for PPS logger filename copy");
    }
    
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
    ESP_LOGI(TAG, "Run for %d hours; GPS on UART2 (TX:17 RX:16 PPS:4). Logs every 1 min -> %s", EXPERIMENT_DURATION_HOURS, filename);

    
    // main monitoring loop - the periodic comparison is handled by rtc_comparison_task
    uint64_t experiment_duration_us = (uint64_t)EXPERIMENT_DURATION_HOURS * 60 * 60 * 1000000;
    uint64_t last_status_time = 0;

    while (1) {
        uint64_t current_time = esp_timer_get_time();
        uint64_t elapsed_time = current_time - experiment_start_time;
        
        if (elapsed_time >= experiment_duration_us) {
            ESP_LOGI(TAG, "Experiment duration reached. Finishing benchmark.");
            break;
        }
        
        // log status every hour
        if (current_time - last_status_time >= 60ULL * 60ULL * 1000000ULL) { 
            uint64_t hours_elapsed = elapsed_time / (60ULL * 60ULL * 1000000ULL);
            uint64_t hours_remaining = EXPERIMENT_DURATION_HOURS - hours_elapsed;
            
            ESP_LOGI(TAG, "Benchmark Status - %llu/%d hours completed, %llu hours remaining, PPS count: %llu", 
                     hours_elapsed, EXPERIMENT_DURATION_HOURS, hours_remaining, pps_count);
            
            // analyze hardware capture performance
            analyze_hardware_captures();

            // sqw visibility: report recent captures and PPS-SQW delta if both seen recently
            if (total_sqw_captures > 0) {
                ESP_LOGI(TAG, "SQW: total=%llu, PPS–SQW phase: %lld us (GPIO %d)",
                         total_sqw_captures, (long long)last_pps_sqw_phase_us, SQW_GPIO_PIN);
            } else {
                ESP_LOGI(TAG, "SQW: no edges captured yet on GPIO %d", SQW_GPIO_PIN);
            }
            
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
