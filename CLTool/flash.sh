#!/usr/bin/env bash

git pull && make && ./cltool -c=/dev/ttyACM0 -uf=./IS_EVB-2_v1.8.2_b3_2020-07-30_165027.hex

if [ ! -d log ]
then
    mkdir log
fi

filetime=$(date '+%Y-%m-%d-%H%M%S')
mv hexdump.log log/$filetime.log
