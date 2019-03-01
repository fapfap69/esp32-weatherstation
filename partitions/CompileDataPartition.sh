#!/bin/bash

python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
--input DeviceData.csv --output DeviceData.bin --size 0x3000

python $IDF_PATH/components/partition_table/gen_esp32part.py \
partitionsV1.csv partitionsV1.bin
