#!/bin/bash

. $IDF_PATH/export.sh

[ -f sdkconfig ] || idf.py set-target esp32c6

idf.py build || exit 1

[ -z "$1" ] || { killall minicom && sleep 2 ; killall -9 minicom && sleep 1 ; idf.py flash -p /dev/ttyACM0 ; }
