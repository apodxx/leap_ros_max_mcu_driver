#include "pid_config.h"

#include <math.h>

#include "nvs.h"

static const char *kPidCfgNamespace = "pid_cfg";
// v2: the transplanted 68000-count chassis uses the STM32 reference gains
// (Kp=7.5 on a 0..1000 PWM scale, Ki=0).  A different key intentionally
// ignores any Kp/Ki/Kd blob saved by the previous chassis/motor firmware, so
// the new defaults actually take effect after the first boot.  Retuning via
// /set_speed_pid is persisted under this key.
static const char *kSpeedPidKey = "speed_v2";

static bool is_valid_pid_config(const PidMsg *config) {
    return config != nullptr &&
           isfinite(config->kp) &&
           isfinite(config->ki) &&
           isfinite(config->kd);
}

esp_err_t pid_load_speed_config(PidMsg *config) {
    if (config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kPidCfgNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t config_size = sizeof(*config);
    err = nvs_get_blob(handle, kSpeedPidKey, config, &config_size);
    if (err == ESP_OK && config_size != sizeof(*config)) {
        err = ESP_ERR_INVALID_SIZE;
    }

    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }

    if (!is_valid_pid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    return err;
}

esp_err_t pid_save_speed_config(const PidMsg *config) {
    if (!is_valid_pid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kPidCfgNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, kSpeedPidKey, config, sizeof(*config));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}
