#!/bin/bash

. $IDF_PATH/export.sh

[ -f sdkconfig ] || idf.py set-target esp32c6
idf.py menuconfig
