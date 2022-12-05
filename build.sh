#!/bin/bash
export IDF_PATH=/home/grobben/develop/libraries/esp-idf
. $IDF_PATH/export.sh
make clean -j8
make -j8
python3 combine.py
