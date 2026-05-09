#!/bin/bash
if [ -n "$IDF_PATH" ] && command -v idf.py >/dev/null 2>&1; then
    echo "ESP-IDF environment is initialized"
else
    echo "ESP-IDF environment is NOT initialized. Running $HOME/esp/esp-idf/export.sh"
    . $HOME/esp/esp-idf/export.sh
fi

idf.py build && idf.py -p /dev/ttyACM0 flash monitor

