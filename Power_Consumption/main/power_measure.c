#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_rtc_time.h"
#include "soc/rtc.h"

static const char *TAG = "deep_sleep";

static void log_clk_sources(void) {
    rtc_slow_freq_t slow = rtc_clk_slow_freq_get();
    const char *slow_str = (slow == RTC_SLOW_FREQ_RTC) ? "Internal RC (150 kHz)" :
                           (slow == RTC_SLOW_FREQ_32K_XTAL) ? "External 32 kHz XTAL" :
                           (slow == RTC_SLOW_FREQ_8MD256) ? "8M/256" : "Unknown";
    ESP_LOGI(TAG, "RTC Slow Clock: %s", slow_str);
}

void app_main(void) {
    log_clk_sources();
    ESP_LOGI(TAG, "Starting power consumption deep sleep setup");
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

    ESP_LOGI(TAG, "Entering deep sleep now");
    esp_deep_sleep_start();
}
