idf_component_register(SRCS "demod1090.c" "mode-s.c" "usb_host_lib_main.c" "demod1090.c" "mode-s.c" "class_driver.c" "esp_libusb.c" "tuner_r82xx.c" "tuner_fc2580.c" "tuner_fc0013.c" "tuner_fc0012.c" "tuner_e4k.c" "librtlsdr.c" 
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES usb esp_driver_gpio
                    )
set_source_files_properties(tuner_r82xx.c "class_driver.c"
    PROPERTIES COMPILE_FLAGS
    "-Wno-unused-function -Wno-unused-variable -Wno-incompatible-pointer-types"
)