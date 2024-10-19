idf_component_register(SRCS "McpwmStepperControl.cpp" 
                    INCLUDE_DIRS "include" "${PROJECT_DIR}/components/Mcpwm/include" "$ENV{IDF_PATH}/components/esp_timer/include" 
                    REQUIRES "driver" "esp_event" "esp_timer")
