set(srcs "FYP_Main_Sensor.c")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES fatfs sd_card
                       REQUIRES driver
                       WHOLE_ARCHIVE)
