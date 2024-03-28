#!/bin/bash


for arg in $@; do
    if [ -f $arg ]; then
        sed "s/[0-9A-Za-z]//g" $arg
    fi

    shift 1
done | sort | uniq

