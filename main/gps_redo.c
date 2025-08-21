#include "gps_redo.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/projdefs.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "sys/time.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <nmea.h>

// sentence parsing is manual for GPRMC.

// timezone offset in hours (denmark 2 hours ahead of UTC)
#define TIMEZONE_OFFSET_HOURS 2

// PPS signal GPIO pin
#define PPS_GPIO_PIN 4
// measurement interval (compare PPS and RTC every 10 minutes)
#define COMPARISON_INTERVAL_MINUTES 10

// UART buffer size
static const int uart_buffer_size = 1024;
static const int GPS_UART_TIMEOUT_MS = 200;     // read timeout
static const int GPS_LOG_EVERY_S = 60;          // throttle GPS info logs

// initialize global variables for time synchronization
volatile uint64_t pps_count = 0;
volatile uint64_t last_pps_time_us = 0;
volatile bool time_synchronized = false;
static struct tm synchronized_time;
static uint64_t sync_timestamp_us = 0;
static FILE* log_file = NULL;

// array to store past PPS timestamps, this is later used for the jitter analysis
#define PPS_HISTORY_SIZE 10
static volatile uint64_t pps_timestamps[PPS_HISTORY_SIZE];
static volatile int pps_history_index = 0;

// common function to record a PPS pulse timestamp
void gps_record_pps_pulse(uint64_t ts_us) {
    last_pps_time_us = ts_us;
    pps_count++;
    // store timestamp in circular buffer for jitter analysis
    pps_timestamps[pps_history_index] = ts_us;
    pps_history_index = (pps_history_index + 1) % PPS_HISTORY_SIZE;
}


// synchronize the time fetched through GPRMC data from NMEA of the GPS module
void synchronize_time_from_gprmc(const char* time_str, const char* date_str) {
    if (!time_synchronized && strlen(time_str) >= 6 && strlen(date_str) >= 6) {
        // parse time HHMMSS
        int hours = (time_str[0] - '0') * 10 + (time_str[1] - '0');
        int minutes = (time_str[2] - '0') * 10 + (time_str[3] - '0');
        int seconds = (time_str[4] - '0') * 10 + (time_str[5] - '0');
        
        // parse date DDMMYY
        int day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
        int month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
        int year = 2000 + (date_str[4] - '0') * 10 + (date_str[5] - '0');
        
        // set up synchronized time structure
        synchronized_time.tm_hour = hours;
        synchronized_time.tm_min = minutes;
        synchronized_time.tm_sec = seconds;
        synchronized_time.tm_mday = day;
        synchronized_time.tm_mon = month - 1;  // tm_mon is 0-based
        synchronized_time.tm_year = year - 1900;  // tm_year is years since 1900
        
        // convert to time_t and set ESP32 RTC
        time_t gps_time = mktime(&synchronized_time);
        struct timeval tv;
        tv.tv_sec = gps_time;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        
        sync_timestamp_us = esp_timer_get_time();
        pps_count = 0;  // reset PPS counter at synchronization
        time_synchronized = true;
        
        ESP_LOGI(GPS_REDO_TAG, "Time synchronized: %04d-%02d-%02d %02d:%02d:%02d UTC", 
                 year, month, day, hours, minutes, seconds);
        ESP_LOGI(GPS_REDO_TAG, "ESP32 RTC synchronized with GPS time");
    }
}

// get current time based on PPS count and initial sync with microsecond precision
void get_pps_time_precise(uint64_t* pps_time_us) {
    if (!time_synchronized) {
        *pps_time_us = 0;
        return;
    }
    
    // calculate time based on PPS count and initial synchronization point
    // each PPS pulse represents exactly 1 second
    uint64_t elapsed_seconds = pps_count;
    
    // convert synchronized time to microseconds since epoch
    time_t sync_time = mktime(&synchronized_time);
    uint64_t sync_time_us = (uint64_t)sync_time * 1000000ULL;
    
    // add elapsed seconds from PPS count
    *pps_time_us = sync_time_us + (elapsed_seconds * 1000000ULL);
}

// get ESP32 RTC time in microseconds
void get_rtc_time_precise(uint64_t* rtc_time_us) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    *rtc_time_us = (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

// calculate time difference in microseconds
int64_t time_diff_us(uint64_t time1_us, uint64_t time2_us) {
    return (int64_t)(time1_us - time2_us);
}

// calculate PPS jitter statistics
void get_pps_jitter_stats(double* avg_interval_us, double* apparent_jitter_us) {
    if (pps_count < 2) {
        *avg_interval_us = 0.0;
        *apparent_jitter_us = 0.0;
        return;
    }
    
    // calculate intervals between recent PPS pulses AS MEASURED BY ESP32 TIMER, so not 100% accurate but should be ~99% accurate
    double intervals[PPS_HISTORY_SIZE - 1];
    int valid_intervals = 0;
    
    for (int i = 0; i < PPS_HISTORY_SIZE - 1 && valid_intervals < (int)(pps_count - 1); i++) {
        int curr_idx = (pps_history_index - 1 - i + PPS_HISTORY_SIZE) % PPS_HISTORY_SIZE;
        int prev_idx = (pps_history_index - 2 - i + PPS_HISTORY_SIZE) % PPS_HISTORY_SIZE;
        
        if (pps_timestamps[curr_idx] > 0 && pps_timestamps[prev_idx] > 0) {
            intervals[valid_intervals] = (double)(pps_timestamps[curr_idx] - pps_timestamps[prev_idx]);
            valid_intervals++;
        }
    }
    
    if (valid_intervals == 0) {
        *avg_interval_us = 0.0;
        *apparent_jitter_us = 0.0;
        return;
    }
    
    // calculate average interval
    double sum = 0.0;
    for (int i = 0; i < valid_intervals; i++) {
        sum += intervals[i];
    }
    *avg_interval_us = sum / valid_intervals;
    
    // calculate apparent jitter
    double variance = 0.0;
    for (int i = 0; i < valid_intervals; i++) {
        double diff = intervals[i] - *avg_interval_us;
        variance += diff * diff;
    }
    *apparent_jitter_us = sqrt(variance / valid_intervals);
}

// log comparison data to SD card and display information on console
void log_time_comparison(const char* filename) {
    if (!time_synchronized) {
        ESP_LOGW(GPS_REDO_TAG, "Time not synchronized yet, skipping comparison");
        return;
    }
    
    // get high-precision timestamps
    uint64_t pps_time_us, rtc_time_us;
    get_pps_time_precise(&pps_time_us);
    get_rtc_time_precise(&rtc_time_us);
    
    // calculate offset in microseconds
    int64_t offset_us = time_diff_us(pps_time_us, rtc_time_us);
    
    // get PPS jitter
    double avg_interval_us, apparent_jitter_us;
    get_pps_jitter_stats(&avg_interval_us, &apparent_jitter_us);
    
    // convert timestamps to readable format for logging
    time_t pps_time_sec = pps_time_us / 1000000ULL;
    time_t rtc_time_sec = rtc_time_us / 1000000ULL;
    uint32_t pps_time_usec = pps_time_us % 1000000ULL;
    uint32_t rtc_time_usec = rtc_time_us % 1000000ULL;
    
    struct tm* pps_tm = gmtime(&pps_time_sec);
    struct tm* rtc_tm = gmtime(&rtc_time_sec);
    
    // log to console
    ESP_LOGI(GPS_REDO_TAG, "Time Comparison [μs precision]:");
    ESP_LOGI(GPS_REDO_TAG, "  PPS: %04d-%02d-%02d %02d:%02d:%02d.%06lu UTC", 
             pps_tm->tm_year + 1900, pps_tm->tm_mon + 1, pps_tm->tm_mday,
             pps_tm->tm_hour, pps_tm->tm_min, pps_tm->tm_sec, pps_time_usec);
    ESP_LOGI(GPS_REDO_TAG, "  RTC: %04d-%02d-%02d %02d:%02d:%02d.%06lu UTC", 
             rtc_tm->tm_year + 1900, rtc_tm->tm_mon + 1, rtc_tm->tm_mday,
             rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec, rtc_time_usec);
    ESP_LOGI(GPS_REDO_TAG, "  Offset: %lld μs (%.3f ms)", offset_us, offset_us / 1000.0);
    ESP_LOGI(GPS_REDO_TAG, "  PPS: Avg=%.1f μs, Jitter=%.1f μs", avg_interval_us, apparent_jitter_us);
    
    // log to SD card
    if (log_file == NULL) {
        log_file = fopen(filename, "a");
        if (log_file != NULL) {
            // write header if file is new
            fseek(log_file, 0, SEEK_END);
            if (ftell(log_file) == 0) {
                fprintf(log_file, "Timestamp_us,PPS_Time_us,RTC_Time_us,Offset_us,Offset_ms,PPS_Count,ESP32_Avg_us,ESP32_Jitter_us\n");
            }
        }
    }
    
    if (log_file != NULL) {
        uint64_t log_timestamp_us = esp_timer_get_time();
        fprintf(log_file, "%llu,%llu,%llu,%lld,%.3f,%llu,%.1f,%.1f\n",
                log_timestamp_us, pps_time_us, rtc_time_us, offset_us, 
                offset_us / 1000.0, pps_count, avg_interval_us, apparent_jitter_us);
        fflush(log_file);
    } else {
        ESP_LOGE(GPS_REDO_TAG, "Failed to open log file on SD card");
    }
}

// convert UTC time to local time string
void convert_utc_to_local(int utc_hours, int utc_minutes, int utc_seconds, 
                         char* local_time_str, size_t str_size) {
    int local_hours = utc_hours + TIMEZONE_OFFSET_HOURS;
    
    // handle day rollover
    if (local_hours >= 24) {
        local_hours -= 24;
    } else if (local_hours < 0) {
        local_hours += 24;
    }
    
    snprintf(local_time_str, str_size, "%02d:%02d:%02d Local", 
             local_hours, utc_minutes, utc_seconds);
}

// simple GPRMC extraction (time/date/status)
static bool parse_gprmc_sentence(const char* sentence,
                                 char raw_time[7],
                                 char raw_date[7],
                                 char* status,
                                 char local_time_str[32],
                                 char date_str[32]) {
    // tokenize without modifying original
    char tmp[256];
    strncpy(tmp, sentence, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    char* tokens[13] = {0};
    int token_count = 0;
    char* tok = strtok(tmp, ",");
    while (tok && token_count < 13) {
        tokens[token_count++] = tok;
        tok = strtok(NULL, ",");
    }

    if (token_count < 9) return false;

    // time
    raw_time[0] = '\0';
    if (tokens[1] && strlen(tokens[1]) >= 6) {
        strncpy(raw_time, tokens[1], 6);
        raw_time[6] = '\0';
        int h = (tokens[1][0]-'0')*10 + (tokens[1][1]-'0');
        int m = (tokens[1][2]-'0')*10 + (tokens[1][3]-'0');
        int s = (tokens[1][4]-'0')*10 + (tokens[1][5]-'0');
        convert_utc_to_local(h, m, s, local_time_str, 32);
    } else {
        local_time_str[0] = '\0';
    }

    // status
    *status = (tokens[2] && tokens[2][0]) ? tokens[2][0] : 'V';

    // date
    raw_date[0] = '\0';
    if (tokens[8] && strlen(tokens[8]) >= 6) {
        strncpy(raw_date, tokens[8], 6);
        raw_date[6] = '\0';
        char day[3] = {tokens[8][0], tokens[8][1], '\0'};
        char mon[3] = {tokens[8][2], tokens[8][3], '\0'};
        char yr[3]  = {tokens[8][4], tokens[8][5], '\0'};
        snprintf(date_str, 32, "%s/%s/20%s", day, mon, yr);
    } else {
        date_str[0] = '\0';
    }
    return true;
}

// checksum guard
static inline bool nmea_has_valid_checksum(const char* sentence) {
    const char* star = strrchr(sentence, '*');
    if (!star || strlen(star) != 3) return false;
    uint8_t calc = nmea_get_checksum(sentence);
    char provided_str[3] = { star[1], star[2], '\0' };
    uint8_t provided = (uint8_t)strtol(provided_str, NULL, 16);
    return calc == provided;
}

QueueHandle_t uart_queue;
void gps_initialize(void){
    // setup UART buffered IO with event queue

    // install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    //configure uart parameters
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // zero-initialize jitter buffer for initial measurements
    for (int i = 0; i < PPS_HISTORY_SIZE; ++i) pps_timestamps[i] = 0;
    
    ESP_LOGI(GPS_REDO_TAG, "GPS and PPS initialization complete");
}

void gps_uart_task(void) {
    ESP_LOGI(GPS_REDO_TAG, "Starting GPS UART task");

    uint8_t* uart_buffer = (uint8_t*) malloc(uart_buffer_size);
    if (uart_buffer == NULL) {
        ESP_LOGE(GPS_REDO_TAG, "Failed to allocate memory for UART buffer");
        vTaskDelete(NULL);
        return;
    }

    char nmea_sentence[256];
    int sentence_pos = 0;
    uint64_t last_gps_log_time = 0;

    while (1) {
    int len = uart_read_bytes(UART_NUM_2, uart_buffer, uart_buffer_size - 1, pdMS_TO_TICKS(GPS_UART_TIMEOUT_MS));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = uart_buffer[i];
                if (c == '$') {
                    sentence_pos = 0;
                    nmea_sentence[sentence_pos++] = c;
                } else if (sentence_pos > 0) {
                    if (c == '\n') {
                        if (sentence_pos > 10) { // minimum valid NMEA length with checksum
                            nmea_sentence[sentence_pos] = '\0';

                            // trim trailing \r
                            if (sentence_pos > 0 && nmea_sentence[sentence_pos - 1] == '\r') {
                                nmea_sentence[sentence_pos - 1] = '\0';
                                sentence_pos--;
                            }
                            // process only GPRMC with valid checksum
                            if (strncmp(nmea_sentence, "$GPRMC", 6) == 0 && nmea_has_valid_checksum(nmea_sentence)) {
                                char raw_time[7], raw_date[7];
                                char status = 'V';
                                char time_str[32] = "";
                                char date_str[32] = "";
                                if (parse_gprmc_sentence(nmea_sentence, raw_time, raw_date, &status, time_str, date_str)) {
                                    if (!time_synchronized && status == 'A' && raw_time[0] && raw_date[0]) {
                                        synchronize_time_from_gprmc(raw_time, raw_date);
                                    }
                                    uint64_t now = esp_timer_get_time();
                                    if (!time_synchronized || (now - last_gps_log_time) >= (uint64_t)GPS_LOG_EVERY_S * 1000000ULL) {
                                        // tokens[3..6]
                                        ESP_LOGI(GPS_REDO_TAG, "GPS - Time: %s, Date: %s, Status: %c", time_str[0]?time_str:"N/A", date_str[0]?date_str:"N/A", status);
                                        last_gps_log_time = now;
                                    }
                                }
                            }
                        }
                        sentence_pos = 0; // reset for next sentence
                    } else if (c != '\r') { // skip \r
                        if (sentence_pos < sizeof(nmea_sentence) - 1) {
                            nmea_sentence[sentence_pos++] = c;
                        } else {
                            sentence_pos = 0;
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(uart_buffer);
    vTaskDelete(NULL);
}

// task for periodic RTC comparison
void rtc_comparison_task(const char* filename) {
    ESP_LOGI(GPS_REDO_TAG, "RTC comparison task started");
    
    while (1) {
        // wait for 10 minutes
        vTaskDelay(pdMS_TO_TICKS(COMPARISON_INTERVAL_MINUTES * 60 * 1000));
        
        // perform time comparison and log
        log_time_comparison(filename);
    }
}

// function to start both GPS and comparison tasks
void start_gps_benchmark_tasks(const char* filename) {
    // start GPS UART task
    xTaskCreate((TaskFunction_t)gps_uart_task, "gps_uart_task", 4096, NULL, 5, NULL);
    
    // start RTC comparison task
    // create a copy of filename to pass to the task since the original might go out of scope
    char* filename_copy = malloc(strlen(filename) + 1);
    if (filename_copy != NULL) {
        strcpy(filename_copy, filename);
        xTaskCreate((TaskFunction_t)rtc_comparison_task, "rtc_comparison_task", 4096, filename_copy, 3, NULL);
    } else {
        ESP_LOGE(GPS_REDO_TAG, "Failed to allocate memory for filename copy");
    }
    
    ESP_LOGI(GPS_REDO_TAG, "GPS benchmark tasks started");
}