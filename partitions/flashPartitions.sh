#!/bin/bash
python /Users/fap/esp/esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 \
--port /dev/cu.SLAB_USBtoUART --baud 115200 --before default_reset --after hard_reset write_flash \
-z --flash_mode dio --flash_freq 40m --flash_size detect \
0x001000 /Users/fap/esp/weatherStation1/build/bootloader/bootloader.bin \
0x008000 /Users/fap/esp/weatherStation1/build/partitionsV1.bin \
0x00d000 /Users/fap/esp/weatherStation1/build/ota_data_initial.bin \
0x010000 /Users/fap/esp/weatherStation1/partitions/DeviceData.bin \
0x120000 /Users/fap/esp/weatherStation1/build/weatherStation1.bin 

