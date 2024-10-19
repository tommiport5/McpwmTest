idf_component_register(SRCS "McpwmTest.cpp"
                    INCLUDE_DIRS "." 
                    PRIV_REQUIRES McpwmStepperControl Mcpwm                     
                    )
