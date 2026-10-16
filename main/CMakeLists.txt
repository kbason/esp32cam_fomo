idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES ei_model_48
                       PRIV_REQUIRES mbedtls)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -O0)                    