#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO 47    // I2C master clock pin
#define I2C_MASTER_SDA_IO 48    // I2C master data pin
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_SLAVE_ADDR 0x28     // I2C address of the slave device

// Calibration handles
static adc_cali_handle_t adc1_cali_handle = NULL;
static adc_cali_handle_t adc2_cali_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle = NULL;

void init_i2c_master() {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &config);
    i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0);
}

void init_adc() {
    adc_cali_curve_fitting_config_t cali_config = {
        .atten = ADC_ATTEN_DB_12, 
        .bitwidth = ADC_WIDTH_BIT_12,
    };

    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
        printf("ADC1 calibration initialized.\n");
    }
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc2_cali_handle) == ESP_OK) {
        printf("ADC2 calibration initialized.\n");
    }

    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    adc_oneshot_unit_init_cfg_t init_config2 = {.unit_id = ADC_UNIT_2};
    adc_oneshot_new_unit(&init_config2, &adc2_handle);

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_WIDTH_BIT_12,
        .atten = ADC_ATTEN_DB_12,
    };

    for (int ch = ADC_CHANNEL_0; ch <= ADC_CHANNEL_7; ch++) {
        adc_oneshot_config_channel(adc1_handle, (adc_channel_t)ch, &channel_config);
        adc_oneshot_config_channel(adc2_handle, (adc_channel_t)ch, &channel_config);
    }
}

void send_data_to_slave(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
}

bool wait_for_ack() {
    uint8_t ack;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &ack, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK && ack == 0xAA);  // Assume 0xAA is the acknowledgment signal
}

void app_main() {
    init_i2c_master();
    init_adc();

    while (1) {
        for (int i = 1; i <= 24; i++) {
            send_data_to_slave(i);

            // Wait for acknowledgment from the slave before proceeding
            if (wait_for_ack()) {
                int64_t current_time = esp_timer_get_time();
                printf("Sent Value: %d, Time: %lld ms", i, current_time / 1000);
                
                // Read ADC1 values
                for (int ch = 0; ch < 8; ch++) {
                    int raw_value;
                    adc_oneshot_read(adc1_handle, (adc_channel_t)ch, &raw_value);
                    printf(", ADC1_CH%d: %d", ch, raw_value);
                }

                // Read ADC2 values
                for (int ch = 0; ch < 8; ch++) {
                    int raw_value;
                    adc_oneshot_read(adc2_handle, (adc_channel_t)ch, &raw_value);
                    printf(", ADC2_CH%d: %d", ch, raw_value);
                }

                printf("\n");
            } else {
                printf("Acknowledgment not received for value: %d\n", i);
            }

            vTaskDelay(pdMS_TO_TICKS(1000));  // Delay before sending next integer
        }
    }
}
