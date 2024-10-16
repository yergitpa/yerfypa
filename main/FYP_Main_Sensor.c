//Original program by Espressif Systems (Apache 2.0 License)
//Modified for EE FYP Project

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <stdlib.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sd_test_io.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#define EXAMPLE_MAX_CHAR_SIZE    64
#define CSV_LINE_COUNT           10  // Number of lines to read from UART and write to SD

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"
#define FILE_PATH   MOUNT_POINT"/sensor_1.csv"
#define FILE_PATH_2   MOUNT_POINT"/sensor_2.csv"

// Pin assignments for SD card
#define PIN_NUM_MISO  21
#define PIN_NUM_MOSI  22
#define PIN_NUM_CLK   23
#define PIN_NUM_CS    18

//For reference TX is white and RX is grey
// UART configuration
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE (1024)

#define UART_NUM_2 LP_UART_NUM_0
#define UART_TX_PIN_2 5
#define UART_RX_PIN_2 4
// #define UART_BAUD_RATE_2 115200
// #define UART_BUF_SIZE_2 (1024)
//Change stuff here


// Shared resources
static SemaphoreHandle_t sdcard_mutex;
static QueueHandle_t uart_queue;

// Function to initialize UART
void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart_queue, 0);
}

void uart_init_2(void)
{ 
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART_TX_PIN_2, UART_RX_PIN_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &uart_queue, 0);
}

// Function to save UART data to SD card
void save_uart_data_to_sd(const char *data)
{
    xSemaphoreTake(sdcard_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "Opening file %s", FILE_PATH);
    FILE *f = fopen(FILE_PATH, "a");  // Open file in append mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        xSemaphoreGive(sdcard_mutex);
        return;
    }

    fprintf(f, "%s\n", data);  // Write the data to the file
    fclose(f);
    ESP_LOGI(TAG, "Sensor Data 1 Saved successfully");
    xSemaphoreGive(sdcard_mutex);
}

void save_uart_data_to_sd_2(const char *data)
{
    xSemaphoreTake(sdcard_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "Opening file %s", FILE_PATH_2);
    FILE *f = fopen(FILE_PATH_2, "a");  // Open file in append mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        xSemaphoreGive(sdcard_mutex);
        return;
    }

    fprintf(f, "%s\n", data);  // Write the data to the file
    fclose(f);
    ESP_LOGI(TAG, "Sensor Data 2 Saved successfully");
    xSemaphoreGive(sdcard_mutex);
}

// FreeRTOS task to read data from UART
void uart_read_task(void *pvParameters)
{
    uart_event_t event;
    char data[EXAMPLE_MAX_CHAR_SIZE];
    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(UART_NUM, (uint8_t *)data, event.size, portMAX_DELAY);
                data[len] = '\0'; // Null-terminate the data
                ESP_LOGI(TAG, "Read from UART: %s", data);
                save_uart_data_to_sd(data); // Save data to SD card
                // save_uart_data_to_sd_2(data); // Save data to SD card_sensor 2
            }
        }
    }
}
void uart_read_task_2(void *pvParameters)
{
    uart_event_t event;
    char data[EXAMPLE_MAX_CHAR_SIZE];
    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(UART_NUM, (uint8_t *)data, event.size, portMAX_DELAY);
                data[len] = '\0'; // Null-terminate the data
                ESP_LOGI(TAG, "Read from UART_2: %s", data);
                // save_uart_data_to_sd(data); // Save data to SD card
                save_uart_data_to_sd_2(data); // Save data to SD card_sensor 2
            }
        }
    }
}

void app_main(void)
{
    esp_err_t ret;
    sdcard_mutex = xSemaphoreCreateMutex();

    // Initialize UART
    uart_init();

    // Options for mounting the filesystem.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SPI peripheral");
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
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Create FreeRTOS task to read from UART
    xTaskCreate(uart_read_task, "uart_read_task", 4096, NULL, 5, NULL);
    xTaskCreate(uart_read_task_2, "uart_read_task_2", 4096, NULL, 5, NULL);

    // Task will continue to run indefinitely
}
