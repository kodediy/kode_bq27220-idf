/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// BQ27220 I2C default address
#define BQ27220_I2C_ADDR                (0x55)

// Standard Commands (mapping aligned with Flipper)
#define BQ27220_REG_CONTROL             0x00
#define BQ27220_REG_TEMPERATURE         0x06 // 0.1 K
#define BQ27220_REG_VOLTAGE             0x08 // mV
#define BQ27220_REG_CURRENT             0x0C // signed mA (device convention: charge negative)
#define BQ27220_REG_AVG_CURRENT         0x14 // signed mA (avg)
#define BQ27220_REG_TIME_TO_EMPTY       0x16 // minutes
#define BQ27220_REG_TIME_TO_FULL        0x18 // minutes
#define BQ27220_REG_STATE_OF_CHARGE     0x2C // %

// Other useful commands
#define BQ27220_REG_REMAINING_CAPACITY  0x10 // mAh
#define BQ27220_REG_FULL_CHG_CAPACITY   0x12 // mAh
#define BQ27220_REG_CYCLE_COUNT         0x2A // cycles
#define BQ27220_REG_STATE_OF_HEALTH     0x2E // %

// Design Capacity command (direct write)
#define BQ27220_REG_DESIGN_CAPACITY     0x3C

// Config Update control subcommands
#define BQ27220_SUBCMD_CONFIG_UPDATE_ENTER    0x0090
#define BQ27220_SUBCMD_CONFIG_UPDATE_EXIT     0x0092
#define BQ27220_SUBCMD_CONFIG_UPDATE_EXIT_RI  0x0091 // exit + reinit

// Data Memory addresses (Profile 1)
#define BQ27220_DM_ADDR_PROFILE1_DESIGN_CAPACITY 0x929F
#define BQ27220_DM_ADDR_PROFILE1_EDV0            0x92B4
#define BQ27220_DM_ADDR_PROFILE1_EDV1            0x92B7
#define BQ27220_DM_ADDR_PROFILE1_EDV2            0x92BA

// Opaque handle pattern
typedef struct bq27220_dev* bq27220_handle_t;

// Initialization using an existing I2C master bus (no ownership of bus)
esp_err_t bq27220_init(i2c_master_bus_handle_t bus, uint8_t i2c_address, uint32_t scl_hz, bq27220_handle_t* out_handle);

// Initialization creating a new I2C master bus (component owns the bus)
esp_err_t bq27220_init_newbus(gpio_num_t sda, gpio_num_t scl, uint32_t scl_hz, bq27220_handle_t* out_handle);

// Delete handle; removes device; deletes bus if owned
esp_err_t bq27220_delete(bq27220_handle_t handle);

// Basic reads
esp_err_t bq27220_read_voltage_mv(bq27220_handle_t handle, int* mv);
esp_err_t bq27220_read_current_ma(bq27220_handle_t handle, int* ma);          // positive = charging
esp_err_t bq27220_read_average_current_ma(bq27220_handle_t handle, int* ma);  // positive = charging
esp_err_t bq27220_read_temperature_c(bq27220_handle_t handle, float* celsius);
esp_err_t bq27220_read_state_of_charge_percent(bq27220_handle_t handle, int* soc);
esp_err_t bq27220_read_time_to_empty_min(bq27220_handle_t handle, int* minutes);
esp_err_t bq27220_read_time_to_full_min(bq27220_handle_t handle, int* minutes);

// Control / security
esp_err_t bq27220_control_subcmd(bq27220_handle_t handle, uint16_t subcmd);
esp_err_t bq27220_unseal(bq27220_handle_t handle, uint16_t key0, uint16_t key1);
esp_err_t bq27220_full_access(bq27220_handle_t handle, uint16_t key0, uint16_t key1);
esp_err_t bq27220_seal(bq27220_handle_t handle);

// Capacities and status
esp_err_t bq27220_read_remaining_capacity_mah(bq27220_handle_t handle, int* mah);
esp_err_t bq27220_read_full_charge_capacity_mah(bq27220_handle_t handle, int* mah);
esp_err_t bq27220_read_cycle_count(bq27220_handle_t handle, int* cycles);
esp_err_t bq27220_read_state_of_health_percent(bq27220_handle_t handle, int* soh);

// Config/DM advanced
esp_err_t bq27220_set_design_capacity_mah(bq27220_handle_t handle, uint16_t mah);
esp_err_t bq27220_begin_config_update(bq27220_handle_t handle);
esp_err_t bq27220_end_config_update(bq27220_handle_t handle, bool reinit);
esp_err_t bq27220_write_data_memory(bq27220_handle_t handle, uint16_t address, const uint8_t* data, uint8_t length);
esp_err_t bq27220_write_data_memory_u16(bq27220_handle_t handle, uint16_t address, uint16_t value);
esp_err_t bq27220_set_edv_raw_u16(bq27220_handle_t handle, uint16_t edv0, uint16_t edv1, uint16_t edv2);
esp_err_t bq27220_set_edvs_mv(bq27220_handle_t handle, uint16_t edv0_mv, uint16_t edv1_mv, uint16_t edv2_mv);

#ifdef __cplusplus
}
#endif


