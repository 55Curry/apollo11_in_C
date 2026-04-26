/**
 * @file main.c
 * @brief Apollo-11 Guidance Computer - ESP32-C3 RV32IMAC Port
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "apollo.h"

static const char *TAG = "APOLLO";

/* External test functions */
extern int test_math_suite(void);
extern int test_vecmath_suite(void);
extern int test_orbit_suite(void);

static void run_apollo_tests(void *pvParameters)
{
    int failures = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Apollo-11 Guidance Computer Test Suite");
    ESP_LOGI(TAG, "  Target: ESP32-C3 RV32IMAC (soft-float)");
    ESP_LOGI(TAG, "========================================\n");

    failures += test_math_suite();
    failures += test_vecmath_suite();
    failures += test_orbit_suite();

    ESP_LOGI(TAG, "========================================");
    if (failures == 0) {
        ESP_LOGI(TAG, "  ALL TESTS PASSED SUCCESSFULLY!");
    } else {
        ESP_LOGI(TAG, "  TEST SUITE FAILED WITH %d ERRORS", failures);
    }
    ESP_LOGI(TAG, "========================================");

    ESP_LOGI(TAG, "Apollo Guidance Computer initialized successfully");

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Apollo-11 ESP32-C3 Port");
    ESP_LOGI(TAG, "FreeRTOS Version: %s", tskKERNEL_VERSION_NUMBER);

    xTaskCreatePinnedToCore(run_apollo_tests, "apollo_tests", 16384, NULL, 5, NULL, 0);
}
