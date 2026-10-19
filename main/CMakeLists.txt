idf_component_register(SRCS "wifi_preview.cpp" "main.cpp" "wifi_preview.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES ei_model_96_noeon esp_http_server esp_wifi nvs_flash esp_netif esp_event
                       PRIV_REQUIRES mbedtls)
target_compile_options(${COMPONENT_LIB} PRIVATE
-Wno-error -O0 
-Wno-error=format 
-Wno-error=unused-but-set-parameter 
"-Wa,-I${CMAKE_CURRENT_SOURCE_DIR}/../models/ei_model_96_noeon"
 )                    