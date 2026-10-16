set(ei_sdk "edge-impulse-sdk")

file(GLOB_RECURSE EI_SOURCES
    "${ei_sdk}/*.cpp"
    "${ei_sdk}/*.c"
    "${ei_sdk}/*.cc"
    "tflite-model/*.cpp"
    "model-parameters/*.cpp"
)

file(GLOB ESP_DSP_INCLUDE_DIRS
    "${ei_sdk}/porting/espressif/esp-dsp/modules/*/include"
)

idf_component_register(
    SRCS ${EI_SOURCES}
    INCLUDE_DIRS
        "."
        "${ei_sdk}"
        "${ei_sdk}/third_party"
        "${ei_sdk}/third_party/flatbuffers/include"
        "${ei_sdk}/third_party/gemmlowp"
        "${ei_sdk}/third_party/ruy"
        "${ei_sdk}/porting/espressif/esp-dsp"
        ${ESP_DSP_INCLUDE_DIRS}
        "model-parameters"
        "tflite-model"
    REQUIRES esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error
    -Wno-error=maybe-uninitialized
    -O3
    -DEIDSP_USE_CMSIS_DSP=0
    -DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=1
)