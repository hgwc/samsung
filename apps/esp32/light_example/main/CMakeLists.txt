idf_component_register(SRCS
                           "main.c"
                            "device_control.c"
                            "iot_cli_cmd.c"
                            "iot_uart_cli.c"
                            "caps_activityLightingMode.c"
                            "caps_colorTemperature.c"
                            "caps_dustSensor.c"
                            "caps_switch.c"
                            "caps_switchLevel.c"
                            "caps_fanSpeed.c"
                           
                            "caps_dustHealthConcern.c"
                            "caps_fineDustHealthConcern.c"
                            "caps_carbonDioxideHealthConcern.c"
                            "caps_carbonDioxideMeasurement.c"
                            "caps_relativeHumidityMeasurement.c"
                            "caps_temperatureMeasurement.c"
                            "caps_signalStrength.c"
                            "caps_tvocMeasurement.c"
                            "caps_tvocHealthConcern.c"
                            "caps_filterStatus.c"
                            "caps_mode.c"
                            "caps_panicAlarm.c"
                            "caps_firmwareUpdate.c"
                           "ota_util.c"                           
                    EMBED_FILES "device_info.json"
                                "onboarding_config.json"
                                "public_key.pem"
                                "root.pem"                                
                    )

set(STDK_IOT_CORE_USE_DEFINED_CONFIG "y")

set(STDK_LINK_LIBRARY
        __idf_libsodium
        __idf_json
   )

set(STDK_INCLUDE_PATH
        "$ENV{IDF_PATH}/components/freertos/include/freertos"
        "$ENV{IDF_PATH}/components/nvs_flash/include"
        "$ENV{IDF_PATH}/components/spi_flash/include"
        "$ENV{IDF_PATH}/components/bootloader_support/include"
    )

add_subdirectory($ENV{STDK_CORE_PATH} iotcore)
target_link_libraries(${COMPONENT_LIB} PUBLIC iotcore)
