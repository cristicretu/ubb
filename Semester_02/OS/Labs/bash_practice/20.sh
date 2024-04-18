#!/bin/bash

if [ $# -lt 1 ]; then
    echo "Usage: $0 <file>"
    exit 1
fi

found_all=false
files=""
while ! ${found_all}; do
    read -p "enter a file" file

    if [ -z "$file" ]; then
        echo "pls non empty"
        continue
    elif [ ! -f "$file" ]; then
        echo "pls file"
        continue
    fi

    files="$files $file"
    found_all=true

    for word in $@; do
        if grep -E -q "\<$word\>" $files; then
            echo "found"
        else
            found_all=false
        fi
    done
done
