idf_component_register(SRCS "McpwmTimer.cpp" "McpwmOperator.cpp" "McpwmGenerator.cpp" "McpwmComparator.cpp"
                    INCLUDE_DIRS "include" "$ENV{IDF_PATH}/components/esp_driver_mcpwm/include/driver"
                    REQUIRES "driver" "esp_timer")

