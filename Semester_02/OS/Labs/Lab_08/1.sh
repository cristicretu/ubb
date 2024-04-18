#!/bin/bash

# EN:
# Write a bash script that takes any number of arguments from the command line.
# For each argument:
#  - if the argument is a directory, list all filenames recursively from that directory that do not contain digits
#  - if the argument is a regular file, print an appropriate message if the file contains at least 20 words (consider that words are any sequence of any characters separated by space)
#

for arg in "$@"; do
    if [ -d "$arg" ]; then
        files=$(find "$arg" -type f)
        for file in $files; do
            filename=$(echo $file | awk -F/ '{print $NF}')
            if ! echo $filename | grep -q '[0-9]'; then
                echo $filename
            fi
        done
    elif [ -f "$arg" ]; then
        lines=$(cat "$arg" | wc -w)
        if [ $lines -ge 20 ]; then
            echo "File $arg contains at least 20 words"
        fi
    fi
done
