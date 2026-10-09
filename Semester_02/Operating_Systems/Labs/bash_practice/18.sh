#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 <directory>"
    exit 1
fi

if [ ! -d $1 ]; then
    echo "$1 is not a directory"
    exit 2
fi

for file in $(find $1 -type f); do
    read_write_for_all=$(ls -l $file)

    if [ $(echo $read_write_for_all | awk :wq  ]; then
        echo "$file is readable for all"
    fi
done
