idf_component_register(
  SRCS "app_main.cpp"
  REQUIRES
    esp_adc
    esp_driver_gpio
    esp_driver_pcnt
    esp_driver_i2c
    esp_timer
    esp_event
  INCLUDE_DIRS "."
)

file(GLOB SOURCE_FILES "*.cpp")
file(GLOB INCLUDE_FILES "*.h")

target_sources(__idf_main
  PRIVATE
  ${SOURCE_FILES} ${INCLUDE_FILES}
)

target_compile_options(__idf_main
  PRIVATE
  -Wno-missing-field-initializers
  -Wno-unused-variable
  -Wno-error=format-truncation
)
