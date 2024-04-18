#!/bin/bash

if [ $# -lt 1 ]; then
    echo "Usage: $0 <file>"
    exit 1
fi

while [ $# -ge 1 ]; do
    if [ -f $1 ]; then
        filetype=$(file $1)
        if echo $filetype | grep -E -q "c program"; then
            for l in $(cat $1 | grep -E "^\s*#include\s+<.*>"); do
                libs+=( $(echo $l) )
            done
        fi
    fi

    shift 1
done

for lib in ${libs[@]}; do
    echo $lib
done
