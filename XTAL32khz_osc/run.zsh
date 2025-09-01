#!/bin/zsh

#create idf.py env
. ~/esp/esp-idf/export.sh
#build project
idf.py build || {echo "Build failed"; exit 1 }

#flash
idf.py flash || {echo "Flash failed"; exit 1 }

#monitor
idf.py monitor
