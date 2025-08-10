/* SPDX-License-Identifier: Apache-2.0 */

#include "bq27220.h"
#include "esp_log.h"

void app_main(void) {
    bq27220_handle_t gauge;
    ESP_ERROR_CHECK(bq27220_init_newbus(48, 47, 400000, &gauge));

    while (1) {
        int mv = 0, ma = 0, soc = 0, ttf = -1;
        float tc = 0.0f;
        bq27220_read_voltage_mv(gauge, &mv);
        bq27220_read_current_ma(gauge, &ma); // positivo = cargando
        bq27220_read_state_of_charge_percent(gauge, &soc);
        bq27220_read_temperature_c(gauge, &tc);
        if (ma > 0 && bq27220_read_time_to_full_min(gauge, &ttf) == ESP_OK) {
            ESP_LOGI("BQ27220", "%d%% | %dmV | %dmA | %.1fC | TTF=%d min", soc, mv, ma, tc, ttf);
        } else {
            ESP_LOGI("BQ27220", "%d%% | %dmV | %dmA | %.1fC", soc, mv, ma, tc);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


