idf_component_register(
    SRCS "main.cpp" "mic_driver.c" "w5500_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer freertos esp_system log
    PRIV_REQUIRES ei_model
)