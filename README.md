# BQ27220 Fuel Gauge

[![Component Registry](https://components.espressif.com/components/kodediy/kode_bq27220/badge.svg)](https://components.espressif.com/components/kodediy/kode_bq27220)

The BQ27220 is a single‑cell Li‑Ion/Li‑Polymer fuel gauge from Texas Instruments. This ESP‑IDF component provides an API to read battery metrics (voltage, current, temperature, state‑of‑charge, time‑to‑empty/full) and perform basic configuration/security operations (unseal/full‑access, design capacity, EDV) over I2C using the ESP‑IDF I2C master driver.

| PMIC | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| BQ27220         | I2C                     | kode_bq27220      | [PDF](https://github.com/kodediy/kode_bq27220/blob/main/BQ27220_Datasheet_RevC.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency kode_bq27220==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

```c
#include "bq27220.h"
#include "esp_log.h"

ESP_LOGI(TAG, "Starting BQ27220 Fuel Gauge initialization");

// Initialize I2C bus
i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = 47, // SCL (example GPIO)
    .sda_io_num = 48, // SDA (example GPIO)
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

// Initialize BQ27220 on existing bus
bq27220_handle_t gauge = NULL;
ESP_ERROR_CHECK(bq27220_init(i2c_bus, BQ27220_I2C_ADDR, 400000, &gauge));
ESP_LOGI(TAG, "BQ27220 initialized");

// Example read
int mv=0, ma=0, soc=0, ttf=0, tte=0;
float tc=0;
bq27220_read_voltage_mv(gauge, &mv);
bq27220_read_current_ma(gauge, &ma); // positive = charging
bq27220_read_state_of_charge_percent(gauge, &soc);
bq27220_read_temperature_c(gauge, &tc);
if (ma > 0 && bq27220_read_time_to_full_min(gauge, &ttf) == ESP_OK) {
    ESP_LOGI("BQ27220", "%d%% | %dmV | %dmA | %.1fC | TTF=%d min", soc, mv, ma, tc, ttf);
} else if (ma <= 0 && bq27220_read_time_to_empty_min(gauge, &tte) == ESP_OK) {
    ESP_LOGI("BQ27220", "%d%% | %dmV | %dmA | %.1fC | TTE=%d min", soc, mv, ma, tc, tte);
} else {
    ESP_LOGI("BQ27220", "%d%% | %dmV | %dmA | %.1fC", soc, mv, ma, tc);
}
```

## Register Reference (BQ27220)

### Standard Commands (primary)
| Address | Name                 | Units/Notes                          |
|--------:|----------------------|---------------------------------------|
|   0x00  | Control()            | 16-bit subcommands (write LSB/MSB)   |
|   0x06  | Temperature          | 0.1 K (Kelvin/10)                    |
|   0x08  | Voltage              | mV                                    |
|   0x0A  | BatteryStatus        | Flags                                  |
|   0x0C  | Current              | Signed mA (device charge = negative)   |
|   0x10  | RemainingCapacity    | mAh                                    |
|   0x12  | FullChargeCapacity   | mAh                                    |
|   0x14  | AverageCurrent       | Signed mA                              |
|   0x16  | TimeToEmpty          | Minutes                                |
|   0x18  | TimeToFull           | Minutes                                |
|   0x2A  | CycleCount           | Cycles                                 |
|   0x2C  | StateOfCharge        | %                                      |
|   0x2E  | StateOfHealth        | %                                      |
|   0x3A  | OperationStatus      | Extended flags                         |
|   0x3C  | DesignCapacity (cmd) | Direct write (U16)                     |

### Data Memory (profile 1 examples)
| Address | Field             | Notes |
|--------:|-------------------|-------|
|  0x929F | DesignCapacity    | U16   |
|  0x92B4 | EDV0              | U16   |
|  0x92B7 | EDV1              | U16   |
|  0x92BA | EDV2              | U16   |