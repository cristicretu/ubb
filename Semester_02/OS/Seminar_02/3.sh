#!/bin/bash


if [ -d $1 ]; then
    for i in $(ls $1); do
        if file $i | grep -E -q "C source"; then
            echo $i
        fi
    done
fi
