idf_component_register(SRCS "discharge.c" 
                #  REQUIRES esp_adc
                 #   REQUIRES esp_driver_gpio
                 #   REQUIRES esp_timer
		         #   REQUIRES esp_driver_i2c
                    INCLUDE_DIRS "." ) 
