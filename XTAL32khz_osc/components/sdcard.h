#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_defs.h>


#define PIN_NUM_MISO  GPIO_NUM_19
#define PIN_NUM_MOSI  GPIO_NUM_23
#define PIN_NUM_CLK   GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_5


#define MOUNT_POINT "/sdcard"
static const char *SD_TAG = "SD Card";


bool file_exists(const char *filename) {
    struct stat st;
    return stat(filename, &st) == 0;
}


int sd_write_index(const char *path){
    FILE *file = fopen(path, "r");

    if (file == NULL) {
        ESP_LOGI(SD_TAG, "Creating file %s", path);
        file = fopen(path, "w");
        fprintf(file, "1");
        return 1;
    } else {
        int current_number;
        fscanf(file, "%d", &current_number);
        fclose(file);

        ESP_LOGI(SD_TAG, "Read number from file: %d", current_number);
        int next_number = current_number + 1;
        file = fopen(path, "w");
        fprintf(file, "%d", next_number);
        fclose(file);
        ESP_LOGI(SD_TAG, "Index incremented");
        return current_number;
    }
}

static esp_err_t sd_write_file_header(const char *path){
    ESP_LOGI(SD_TAG, "Creating headers in data.csv...");
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, "Timestamp, PPS_Time, RTC_Time, Offset_us, Offset_ms, PPS_Count, PPS_Avg_us, PPS_Jitter_us\n");
    fclose(f);
    ESP_LOGI(SD_TAG, "Headers in data.csv created.");
    return ESP_OK;
}

static esp_err_t sd__write_file(const char *path, char ntptime[64], char rtctime[64], double offset, int delay_ms){
    ESP_LOGI(SD_TAG, "Opening file %s", path);
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, "%s, %s, %.6f, %d \n", ntptime, rtctime, offset, delay_ms);
    fclose(f);
    ESP_LOGI(SD_TAG, "File written");

    return ESP_OK;
}

void sdcardinitialize(void){
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(SD_TAG, "Initializing SD card");

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
    if(ret != ESP_OK){
        ESP_LOGE(SD_TAG, "Failed to initialize SPI bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(SD_TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(SD_TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(SD_TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(SD_TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    const char *data_file = MOUNT_POINT"/data.csv";
}
