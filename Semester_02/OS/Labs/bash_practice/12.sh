#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

if [ ! -d $1 ]; then
    echo "the directory does not exist"
    exit 1
fi

no_files=$(find "$1" -type f | wc -l)
while true; do
    if [ ! -d $1  ]; then
        echo "the directory has been deleted bor!!!!!"
        break
    fi

    no_files_now=$(find "$1" -type f | wc -l)

    if [ $no_files_now -ne $no_files ]; then
        echo "Number of files in the directory has changed"
        no_files=$no_files_now
    fi
    sleep 1
done
