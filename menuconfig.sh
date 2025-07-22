#!/bin/bash

. $IDF_PATH/export.sh

[ -f sdkconfig ] || idf.py set-target esp32s3
idf.py menuconfig
