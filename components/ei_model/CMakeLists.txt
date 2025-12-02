cmake_minimum_required(VERSION 3.5)

# 1. Find all source files recursively (Including .cc for TensorFlow!)
file(GLOB_RECURSE SDK_SOURCES 
    "edge-impulse-sdk/classifier/*.cpp"
    "edge-impulse-sdk/classifier/*.c"
    "edge-impulse-sdk/dsp/*.cpp"
    "edge-impulse-sdk/dsp/*.c"
    "edge-impulse-sdk/porting/*.cpp"
    "edge-impulse-sdk/tensorflow/*.cpp"
    "edge-impulse-sdk/tensorflow/*.c"
    "edge-impulse-sdk/tensorflow/*.cc"
    "edge-impulse-sdk/third_party/*.cpp"
    "edge-impulse-sdk/third_party/*.c"
    "edge-impulse-sdk/third_party/*.cc"
    "model-parameters/*.cpp"
    "tflite-model/*.cpp"
    "tflite-model/*.cc"
)

# 2. Register the component
idf_component_register(
    SRCS ${SDK_SOURCES} "ei_porting.cpp"
    INCLUDE_DIRS "."
                 "edge-impulse-sdk"
                 "edge-impulse-sdk/classifier"
                 "edge-impulse-sdk/dsp"
                 "edge-impulse-sdk/dsp/kissfft"
                 "edge-impulse-sdk/dsp/dct"
                 "edge-impulse-sdk/porting"
                 "edge-impulse-sdk/third_party"
                 "model-parameters"
                 "tflite-model"
    REQUIRES nvs_flash esp_timer freertos esp_system log esp-dsp
)

# 3. Suppress warnings and prevent them from stopping the build
component_compile_options(
    -Wno-strict-aliasing 
    -Wno-type-limits 
    -Wno-unused-parameter 
    -Wno-missing-field-initializers 
    -Wno-error=maybe-uninitialized
    -Wno-error=unused-variable
    -Wno-error=unused-but-set-variable
    -Wno-error=unused-function
)