/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "bq27220.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char* TAG = "bq27220";

#define I2C_DEFAULT_PORT I2C_NUM_0

typedef struct bq27220_dev {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    bool owns_bus;
    uint8_t i2c_address;
} bq27220_dev_t;

static esp_err_t add_device(bq27220_dev_t* dev_ctx, uint32_t scl_hz) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = dev_ctx->i2c_address,
        .scl_speed_hz    = (scl_hz ? scl_hz : 400000),
    };
    return i2c_master_bus_add_device(dev_ctx->bus, &dev_cfg, &dev_ctx->dev);
}

// --- Init on existing bus ---
esp_err_t bq27220_init(i2c_master_bus_handle_t bus, uint8_t i2c_address, uint32_t scl_hz, bq27220_handle_t* out_handle) {
    ESP_RETURN_ON_FALSE(bus && out_handle, ESP_ERR_INVALID_ARG, TAG, "bad args");
    bq27220_dev_t* dev = (bq27220_dev_t*)calloc(1, sizeof(bq27220_dev_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no mem");
    dev->bus = bus;
    dev->i2c_address = i2c_address ? i2c_address : BQ27220_I2C_ADDR;
    dev->owns_bus = false;
    esp_err_t err = add_device(dev, scl_hz);
    if (err != ESP_OK) {
        free(dev);
        return err;
    }
    // probe by reading voltage
    int mv = 0;
    err = bq27220_read_voltage_mv((bq27220_handle_t)dev, &mv);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(dev->dev);
        free(dev);
        return err;
    }
    *out_handle = (bq27220_handle_t)dev;

    ESP_LOGI(TAG, "BQ27220 initialized successfully, version: %d.%d.%d", KODE_BQ27220_VER_MAJOR, KODE_BQ27220_VER_MINOR, KODE_BQ27220_VER_PATCH);

    return ESP_OK;
}

// --- Init creating new bus ---
esp_err_t bq27220_init_newbus(gpio_num_t sda, gpio_num_t scl, uint32_t scl_hz, bq27220_handle_t* out_handle) {
    ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "out ptr null");
    bq27220_dev_t* dev = (bq27220_dev_t*)calloc(1, sizeof(bq27220_dev_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no mem");
    dev->i2c_address = BQ27220_I2C_ADDR;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_DEFAULT_PORT,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true }
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &dev->bus);
    if (err != ESP_OK) {
        free(dev);
        return err;
    }
    dev->owns_bus = true;
    err = add_device(dev, scl_hz);
    if (err != ESP_OK) {
        i2c_del_master_bus(dev->bus);
        free(dev);
        return err;
    }
    // probe
    int mv = 0;
    err = bq27220_read_voltage_mv((bq27220_handle_t)dev, &mv);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(dev->dev);
        i2c_del_master_bus(dev->bus);
        free(dev);
        return err;
    }
    *out_handle = (bq27220_handle_t)dev;

    ESP_LOGI(TAG, "BQ27220 initialized successfully, version: %d.%d.%d", KODE_BQ27220_VER_MAJOR, KODE_BQ27220_VER_MINOR, KODE_BQ27220_VER_PATCH);
    return ESP_OK;
}

esp_err_t bq27220_delete(bq27220_handle_t handle) {
    if (!handle) return ESP_OK;
    bq27220_dev_t* dev = (bq27220_dev_t*)handle;
    if (dev->dev) {
        i2c_master_bus_rm_device(dev->dev);
        dev->dev = NULL;
    }
    if (dev->owns_bus && dev->bus) {
        i2c_del_master_bus(dev->bus);
        dev->bus = NULL;
    }
    free(dev);
    return ESP_OK;
}

// ---- low-level helpers ----
static esp_err_t read_u16(bq27220_handle_t handle, uint8_t reg, uint16_t* out) {
    bq27220_dev_t* dev = (bq27220_dev_t*)handle;
    uint8_t rx[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(dev->dev, &reg, 1, rx, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) return err;
    *out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8); // little-endian
    return ESP_OK;
}

static esp_err_t write_u16_to_reg00(bq27220_handle_t handle, uint16_t val) {
    bq27220_dev_t* dev = (bq27220_dev_t*)handle;
    uint8_t tx[3];
    tx[0] = BQ27220_REG_CONTROL;
    tx[1] = (uint8_t)(val & 0xFF);
    tx[2] = (uint8_t)(val >> 8);
    return i2c_master_transmit(dev->dev, tx, sizeof(tx), pdMS_TO_TICKS(100));
}

// ---- public API ----
esp_err_t bq27220_read_voltage_mv(bq27220_handle_t handle, int* mv) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_VOLTAGE, &raw), TAG, "i2c");
    *mv = (int)raw;
    return ESP_OK;
}

// NOTE: device convention is charge = negative; we flip to "positive = charging"
esp_err_t bq27220_read_current_ma(bq27220_handle_t handle, int* ma) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_CURRENT, &raw), TAG, "i2c");
    int16_t s = (int16_t)raw;
    *ma = -((int)s);
    return ESP_OK;
}

esp_err_t bq27220_read_average_current_ma(bq27220_handle_t handle, int* ma) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_AVG_CURRENT, &raw), TAG, "i2c");
    int16_t s = (int16_t)raw;
    *ma = -((int)s);
    return ESP_OK;
}

esp_err_t bq27220_read_temperature_c(bq27220_handle_t handle, float* celsius) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_TEMPERATURE, &raw), TAG, "i2c");
    float kelvin = ((float)raw) / 10.0f;
    *celsius = kelvin - 273.15f;
    return ESP_OK;
}

esp_err_t bq27220_read_state_of_charge_percent(bq27220_handle_t handle, int* soc) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_STATE_OF_CHARGE, &raw), TAG, "i2c");
    *soc = (int)raw;
    return ESP_OK;
}

esp_err_t bq27220_read_time_to_empty_min(bq27220_handle_t handle, int* minutes) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_TIME_TO_EMPTY, &raw), TAG, "i2c");
    *minutes = (int)(int16_t)raw; // minutes
    return ESP_OK;
}

esp_err_t bq27220_read_time_to_full_min(bq27220_handle_t handle, int* minutes) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_TIME_TO_FULL, &raw), TAG, "i2c");
    *minutes = (int)(int16_t)raw; // minutes
    return ESP_OK;
}

// Control
esp_err_t bq27220_control_subcmd(bq27220_handle_t handle, uint16_t subcmd) {
    return write_u16_to_reg00(handle, subcmd);
}

esp_err_t bq27220_unseal(bq27220_handle_t handle, uint16_t key0, uint16_t key1) {
    ESP_RETURN_ON_ERROR(write_u16_to_reg00(handle, key0), TAG, "key0");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(write_u16_to_reg00(handle, key1), TAG, "key1");
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t bq27220_full_access(bq27220_handle_t handle, uint16_t key0, uint16_t key1) {
    ESP_RETURN_ON_ERROR(write_u16_to_reg00(handle, key0), TAG, "key0");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(write_u16_to_reg00(handle, key1), TAG, "key1");
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t bq27220_seal(bq27220_handle_t handle) {
    return bq27220_control_subcmd(handle, 0x0020);
}

// ---- Extra measurements ----
esp_err_t bq27220_read_remaining_capacity_mah(bq27220_handle_t handle, int* mah) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_REMAINING_CAPACITY, &raw), TAG, "i2c");
    *mah = (int)raw;
    return ESP_OK;
}

esp_err_t bq27220_read_full_charge_capacity_mah(bq27220_handle_t handle, int* mah) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_FULL_CHG_CAPACITY, &raw), TAG, "i2c");
    *mah = (int)raw;
    return ESP_OK;
}

esp_err_t bq27220_read_cycle_count(bq27220_handle_t handle, int* cycles) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_CYCLE_COUNT, &raw), TAG, "i2c");
    *cycles = (int)raw;
    return ESP_OK;
}

esp_err_t bq27220_read_state_of_health_percent(bq27220_handle_t handle, int* soh) {
    uint16_t raw;
    ESP_RETURN_ON_ERROR(read_u16(handle, BQ27220_REG_STATE_OF_HEALTH, &raw), TAG, "i2c");
    *soh = (int)raw;
    return ESP_OK;
}

// ---- Advanced config / DM ----
esp_err_t bq27220_set_design_capacity_mah(bq27220_handle_t handle, uint16_t mah) {
    bq27220_dev_t* dev = (bq27220_dev_t*)handle;
    uint8_t tx[3];
    tx[0] = BQ27220_REG_DESIGN_CAPACITY;
    tx[1] = (uint8_t)(mah & 0xFF);
    tx[2] = (uint8_t)(mah >> 8);
    return i2c_master_transmit(dev->dev, tx, sizeof(tx), pdMS_TO_TICKS(100));
}

esp_err_t bq27220_begin_config_update(bq27220_handle_t handle) {
    return bq27220_control_subcmd(handle, BQ27220_SUBCMD_CONFIG_UPDATE_ENTER);
}

esp_err_t bq27220_end_config_update(bq27220_handle_t handle, bool reinit) {
    return bq27220_control_subcmd(handle, reinit ? BQ27220_SUBCMD_CONFIG_UPDATE_EXIT_RI
                                                 : BQ27220_SUBCMD_CONFIG_UPDATE_EXIT);
}

esp_err_t bq27220_write_data_memory(bq27220_handle_t handle, uint16_t address, const uint8_t* data, uint8_t length) {
    bq27220_dev_t* dev = (bq27220_dev_t*)handle;
    ESP_RETURN_ON_FALSE(dev && data && length && length <= 32, ESP_ERR_INVALID_ARG, TAG, "args");

    uint8_t tx[2];
    tx[0] = 0x3E; tx[1] = (uint8_t)(address & 0xFF);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev->dev, tx, sizeof(tx), pdMS_TO_TICKS(100)), TAG, "3E");
    tx[0] = 0x3F; tx[1] = (uint8_t)((address >> 8) & 0xFF);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev->dev, tx, sizeof(tx), pdMS_TO_TICKS(100)), TAG, "3F");

    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t buf[1 + 32];
    buf[0] = 0x40;
    memcpy(&buf[1], data, length);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev->dev, buf, 1 + length, pdMS_TO_TICKS(100)), TAG, "data");

    uint8_t sum = (uint8_t)(address & 0xFF) + (uint8_t)((address >> 8) & 0xFF);
    for (uint8_t i = 0; i < length; ++i) sum += data[i];
    uint8_t csum = (uint8_t)~sum;

    uint8_t cs[3];
    cs[0] = 0x60; cs[1] = csum; cs[2] = (uint8_t)(length + 4);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev->dev, cs, sizeof(cs), pdMS_TO_TICKS(100)), TAG, "csum");

    vTaskDelay(pdMS_TO_TICKS(4));
    return ESP_OK;
}

esp_err_t bq27220_write_data_memory_u16(bq27220_handle_t handle, uint16_t address, uint16_t value) {
    uint8_t buf[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
    return bq27220_write_data_memory(handle, address, buf, 2);
}

esp_err_t bq27220_set_edv_raw_u16(bq27220_handle_t handle, uint16_t edv0, uint16_t edv1, uint16_t edv2) {
    ESP_RETURN_ON_ERROR(bq27220_write_data_memory_u16(handle, BQ27220_DM_ADDR_PROFILE1_EDV0, edv0), TAG, "edv0");
    ESP_RETURN_ON_ERROR(bq27220_write_data_memory_u16(handle, BQ27220_DM_ADDR_PROFILE1_EDV1, edv1), TAG, "edv1");
    ESP_RETURN_ON_ERROR(bq27220_write_data_memory_u16(handle, BQ27220_DM_ADDR_PROFILE1_EDV2, edv2), TAG, "edv2");
    return ESP_OK;
}

esp_err_t bq27220_set_edvs_mv(bq27220_handle_t handle, uint16_t edv0_mv, uint16_t edv1_mv, uint16_t edv2_mv) {
    return bq27220_set_edv_raw_u16(handle, edv0_mv, edv1_mv, edv2_mv);
}

