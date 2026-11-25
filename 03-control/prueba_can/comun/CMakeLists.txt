idf_component_register(SRCS "prueba_can.c" "robot.c" "robot_sim.c"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS "../../pata_MOD_DRV/main"   # can_protocol.h, compartido con la pata
                       REQUIRES driver esp_driver_gpio esp_timer nvs_flash)
