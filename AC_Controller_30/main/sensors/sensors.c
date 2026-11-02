#include "main.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sensors.h"
#include "sensirion_gas_index_algorithm.h"


#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000  

SemaphoreHandle_t i2c_mutex;
GasIndexAlgorithmParams voc_params;
GasIndexAlgorithmParams nox_params;

uint8_t sgp_crc(uint8_t *data, int len);

esp_err_t i2c_write(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    if (i2c_mutex == NULL) {
        ESP_LOGE("I2C", "Mutex not initialized");
        return ESP_FAIL;
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
   esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr,
                                      (uint8_t[]){reg, data}, 2,
                                      pdMS_TO_TICKS(100));

    xSemaphoreGive(i2c_mutex);
    return err;
}

esp_err_t i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (i2c_mutex == NULL) {
        ESP_LOGE("I2C", "Mutex not initialized");
        return ESP_FAIL;
    }
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t err =  i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr,
                                        &reg, 1,
                                        data, len,
                                        pdMS_TO_TICKS(100));
    xSemaphoreGive(i2c_mutex);
    return err;
}


void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    // Sensor initialization (example for BME680)
    i2c_write(0x76, 0x72, 0x01); // humidity OS
    i2c_write(0x76, 0x74, 0x25); // temp OS + forced mode
    //PRECISION_LIGHT (OPT4048) configuration
    uint8_t config[3];
    // Register 0x01 = configuration
    config[0] = 0x01;
    // Example config:
    // continuous conversion, auto range
    config[1] = 0xC4;
    config[2] = 0x10;

    i2c_master_write_to_device(I2C_MASTER_NUM, 0x45, config, 3, 100);
    vTaskDelay(pdMS_TO_TICKS(150));
    GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithm_init(&nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
}


void Read_sensor(uint8_t ucchoose_sensor)
{   uint8_t data[6];
    uint8_t cmd[10];
    int32_t voc_index;
    int32_t nox_index;
    switch (ucchoose_sensor)
    {
        case PRECISION_LIGHT:   //OPT4048
            i2c_read(0x45, 0x00, data, 2);

            uint16_t result = (data[0] << 8) | data[1];
            printf("RAW HEX: %02X %02X\n", data[0], data[1]);
            ESP_LOGI("SENSOR","PRECISION_LIGHT (OPT4048) %d\n",result);
        break;
        case TEMP_HUMIDITY:   //BME680

            int32_t var1, var2, t_fine;
            float temp_1;

            int16_t par_t1;
            int16_t par_t2;
            int8_t par_t3;
            i2c_write(0x76, 0x74, 0x25);

            vTaskDelay(pdMS_TO_TICKS(50));

            i2c_read(0x76, 0x22, data, 3);

            uint8_t buf[3];

            // par_t1
            i2c_read(0x76, 0xE9, buf, 2);
            par_t1 = (uint16_t)(buf[1] << 8 | buf[0]);

            // par_t2
            i2c_read(0x76, 0x8A, buf, 2);
            par_t2 = (int16_t)(buf[1] << 8 | buf[0]);

            // par_t3
            i2c_read(0x76, 0x8C, buf, 1);
            par_t3 = (int8_t)buf[0];

            int32_t temp_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
            printf("RAW: %02X %02X %02X\n", data[0], data[1], data[2]);

            var1 = (((temp_raw >> 3) - ((int32_t)par_t1 << 1)) * ((int32_t)par_t2)) >> 11;

            var2 = (((((temp_raw >> 4) - ((int32_t)par_t1)) *
                    ((temp_raw >> 4) - ((int32_t)par_t1))) >> 12) *
                    ((int32_t)par_t3)) >> 14;

            t_fine = var1 + var2;

            temp_1 = (t_fine * 5 + 128) >> 8; // °C * 100

            temp_1/=100; // Convert to °C
            ESP_LOGI("SENSOR","TEMP (BME680): %f\n",temp_1);
            break;
        case THREE_AXIS_MAGNETOMETER:  //BMM350

            i2c_read(0x10, 0x42, data, 6);

            int16_t mag_x = (data[1] << 8) | data[0];
            int16_t mag_y = (data[3] << 8) | data[2];
            int16_t mag_z = (data[5] << 8) | data[4];
            ESP_LOGI("SENSOR","THREE_AXIS_MAGNETOMETER (BMM350) X:%d Y:%d Z:%d\n",mag_x,mag_y,mag_z);
        break;
        case HIGH_ACC_HUMIDITY_TEMP:  //SHT45
            cmd[0] = 0xFD; // High precision measurement
            i2c_master_write_to_device(I2C_MASTER_NUM, 0x44, cmd, 1, 100);

            vTaskDelay(pdMS_TO_TICKS(10));
            i2c_master_read_from_device(I2C_MASTER_NUM, 0x44, data, 6, 100);

            // Convert
            float temp = -45 + 175 * ((data[0] << 8 | data[1]) / 65535.0);
            float hum  = 100 * ((data[3] << 8 | data[4]) / 65535.0);
            ESP_LOGI("SENSOR","HIGH_ACC_HUMIDITY_TEMP(SHT45): Temp:%f, HUM:%f\n",temp,hum);
        break;
        case AIR_QUALITY_SENSOR: //SGP4x
            cmd[0] = 0x26;
            cmd[1] = 0x0F; // Measure raw signal
            i2c_master_write_to_device(I2C_MASTER_NUM, 0x59, cmd, 2, 100);

            vTaskDelay(pdMS_TO_TICKS(100));

            i2c_master_read_from_device(I2C_MASTER_NUM, 0x59, data, 6, 100);
            printf("RAW: %02X %02X %02X,%02X %02X %02X\n", data[0], data[1], data[2],data[3], data[4], data[5]);
            if (sgp_crc(data, 2) != data[2]||sgp_crc(data+3, 2) != data[5]) {
                ESP_LOGE("SENSOR", "CRC error in AIR_QUALITY_SENSOR(SGP4x)");
            } else {
                uint16_t sraw_voc = (data[0] << 8) | data[1];
                uint16_t sraw_nox = (data[3] << 8) | data[4];
                ESP_LOGI("SENSOR","AIR_QUALITY_SENSOR(SGP4x): AW VOC:%d, RAW NOx:%d \n",sraw_voc,sraw_nox);
                GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index);
                GasIndexAlgorithm_process(&nox_params, sraw_nox, &nox_index);

                printf("VOC Index: %ld\n", voc_index);
                printf("NOx Index: %ld\n", nox_index);
                // VOC Index:
                // 0–100   → Good
                // 100–200 → Moderate
                // 200+    → Poor

                // NOx Index:
                // <100 → Clean air
                // >200 → polluted
            }
        break;
        case TIME_OF_FLIGHT_DISTANCE:  //VL53L1CX
        break;
        case NINE_AXIS_ORIENTATION_SENSOR:  //BNO086
        break;
    }

}


uint8_t sgp_crc(uint8_t *data, int len)
{
    uint8_t crc = 0xFF;

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void Read_sensor_task(void *arg) {

  while (1) {
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}