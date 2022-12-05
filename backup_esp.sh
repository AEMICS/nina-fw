#!/bin/bash
export IDF_PATH=/home/grobben/develop/libraries/esp-idf
. $IDF_PATH/export.sh
python3 $IDF_PATH/components/esptool_py/esptool/esptool.py -b 115200 --port /dev/ttyUSB0 read_flash 0x000000 0x200000 flash_2M.bin 
