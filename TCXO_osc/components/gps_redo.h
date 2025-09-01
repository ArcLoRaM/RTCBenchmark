#ifndef GPS_REDO
#define GPS_REDO
#include "driver/uart.h"
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

static const char *GPS_REDO_TAG = "GPS_REDO";

// Global variables declarations
extern volatile uint64_t pps_count;
extern volatile uint64_t last_pps_time_us;
extern volatile bool time_synchronized;

void gps_initialize(void);
void gps_uart_task(void);
// Record a PPS pulse timestamp (in microseconds). Populated by hardware capture ISR.
void gps_record_pps_pulse(uint64_t ts_us);
void synchronize_time_from_gprmc(const char* time_str, const char* date_str);
void get_pps_time_precise(uint64_t* pps_time_us);
void get_rtc_time_precise(uint64_t* rtc_time_us);
int64_t time_diff_us(uint64_t time1_us, uint64_t time2_us);
void log_time_comparison(const char* filename);
void rtc_comparison_task(const char* filename);
void start_gps_benchmark_tasks(const char* filename);

#endif //
