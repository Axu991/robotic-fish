/**
 * @file adc_driver.c
 * @brief ADC 驱动实现（模拟信号采集）
 * @author huangxu <huangxu2024@shanghaitech.edu.cn>
 * @date 2026-07-30
 * @version 1.0.0
 */


#include "adc_driver.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "ADC_DRIVER";

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;
static bool cali_enabled = false;
static adc_unit_t adc_unit = ADC_UNIT_1;
static adc_channel_t adc_channel = ADC_CHANNEL_0;

void adc_init(gpio_num_t pin)
{
    esp_err_t ret = adc_oneshot_io_to_channel(pin, &adc_unit, &adc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d is not ADC capable: %s", pin, esp_err_to_name(ret));
        return;
    }

    // 配置 ADC 单元
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_cfg, &adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC unit initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    // 配置 ADC 通道
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 12 位分辨率
        .atten = ADC_ATTEN_DB_12, // 12dB 衰减，输入电压范围为 0-3.9V
    };
    ret = adc_oneshot_config_channel(adc_handle, adc_channel, &chan_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC channel configuration failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        return;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = adc_unit,
        .chan = adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    cali_enabled = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK);
#endif

    ESP_LOGI(TAG, "ADC initialized on GPIO%d (unit %d, channel %d, calibration %s)",
             pin, adc_unit, adc_channel, cali_enabled ? "enabled" : "unavailable");
}

int adc_read_millvoltage(void)
{
    if (adc_handle == NULL)
    {
        ESP_LOGE(TAG, "ADC not initialized");
        return -1; // ADC 未初始化
    }

    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, adc_channel, &raw);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return -1; // 读取失败
    }

    int voltage_mv = 0;
    if (cali_enabled){
        ret = adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "ADC calibration failed: %s", esp_err_to_name(ret));
            return -1; // 校准失败
        }
    }
    else
    {
        voltage_mv = (raw * 3300) / 4095; // 3.3V 对应 4095
    }
    return voltage_mv;
}

float adc_read_voltage(void)
{
    int millivoltage = adc_read_millvoltage();
    if (millivoltage < 0)
    {
        return -1.0f; // 读取失败
    }
    return millivoltage / 1000.0f; // 转换为伏特
}
