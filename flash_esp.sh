#!/bin/bash
export IDF_PATH=/home/grobben/develop/libraries/esp-idf
. $IDF_PATH/export.sh
# expecting USB 2 serial on ttyUSB0
python3 $IDF_PATH/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before no_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x0 NINA_W102.bin
