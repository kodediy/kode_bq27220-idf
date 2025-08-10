/* SPDX-License-Identifier: Apache-2.0 */

#include "bq27220.h"
#include "esp_log.h"

#define BATTERY_CAPACITY_MAH 500

void app_main(void) {
    bq27220_handle_t gauge;
    ESP_ERROR_CHECK(bq27220_init_newbus(48, 47, 400000, &gauge));

    // Ejemplo avanzado: configurar capacidad y (opcional) EDVs vía DM/Config Update
    // NOTA: Para DM suele requerirse UNSEAL / FULL ACCESS con tus claves reales.
    // Sustituye 0x0414/0x3672 por tus claves si procede.
    (void)bq27220_unseal(gauge, 0x0414, 0x3672);
    (void)bq27220_full_access(gauge, 0xFFFF, 0xFFFF);

    // Set Design Capacity por comando directo
    ESP_ERROR_CHECK(bq27220_set_design_capacity_mah(gauge, BATTERY_CAPACITY_MAH));

    // (Opcional) Entrar en CONFIG_UPDATE y escribir EDVs en mV
    //ESP_ERROR_CHECK(bq27220_begin_config_update(gauge));
    //ESP_ERROR_CHECK(bq27220_set_edvs_mv(gauge, 3200, 3300, 3400));
    //ESP_ERROR_CHECK(bq27220_end_config_update(gauge, true));

    while (1) {
        int mv=0, ma=0, soc=0, tte=0, ttf=0, mah_rem=0, mah_fcc=0, cycles=0, soh=0;
        float tc=0;
        bq27220_read_voltage_mv(gauge, &mv);
        bq27220_read_current_ma(gauge, &ma);
        bq27220_read_state_of_charge_percent(gauge, &soc);
        bq27220_read_temperature_c(gauge, &tc);
        bq27220_read_time_to_empty_min(gauge, &tte);
        bq27220_read_time_to_full_min(gauge, &ttf);
        bq27220_read_remaining_capacity_mah(gauge, &mah_rem);
        bq27220_read_full_charge_capacity_mah(gauge, &mah_fcc);
        bq27220_read_cycle_count(gauge, &cycles);
        bq27220_read_state_of_health_percent(gauge, &soh);

        if (ma > 0) {
            ESP_LOGI("BQ27220",
                     "%d%% | %dmV | %dmA | %.1fC | TTF=%d min | Rem=%dmAh | FCC=%dmAh | Cyc=%d | SOH=%d%%",
                     soc, mv, ma, tc, ttf, mah_rem, mah_fcc, cycles, soh);
        } else {
            ESP_LOGI("BQ27220",
                     "%d%% | %dmV | %dmA | %.1fC | TTE=%d min | Rem=%dmAh | FCC=%dmAh | Cyc=%d | SOH=%d%%",
                     soc, mv, ma, tc, tte, mah_rem, mah_fcc, cycles, soh);
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}


