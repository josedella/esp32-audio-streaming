/* Edge Impulse Porting Layer for ESP32 */
#include <stdlib.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#define EI_WEAK_FN __attribute__((weak))

EI_WEAK_FN EI_IMPULSE_ERROR ei_run_impulse_check_canceled() {
    return EI_IMPULSE_OK;
}

EI_WEAK_FN EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
    vTaskDelay(time_ms / portTICK_PERIOD_MS);
    return EI_IMPULSE_OK;
}

uint64_t ei_read_timer_ms() {
    return esp_timer_get_time() / 1000;
}

uint64_t ei_read_timer_us() {
    return esp_timer_get_time();
}

void ei_printf(const char *format, ...) {
    va_list myargs;
    va_start(myargs, format);
    esp_log_writev(ESP_LOG_INFO, "EI", format, myargs);
    va_end(myargs);
}

void ei_printf_float(float f) {
    ei_printf("%f", f);
}

void *ei_malloc(size_t size) {
    return malloc(size);
}

void *ei_calloc(size_t nitems, size_t size) {
    return calloc(nitems, size);
}

void ei_free(void *ptr) {
    free(ptr);
}

#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C"
#endif
void DebugLog(const char* s) {
    ei_printf("%s", s);
}