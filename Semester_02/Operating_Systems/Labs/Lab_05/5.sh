#!/bin/bash
# pairs of filenamd and word
# for every pair print an appropiate message if the word appears in the file at least 3 times
#

if [ $# -lt 2 ]; then
    echo "Usage: $0 <filename> <word>"
    exit 1
fi

while [ $# -gt 0 ]; do
    filename=$1
    word=$2

    if [ ! -f $filename ]; then
        echo "Error: $filename is not a file"
        shift 2
        continue
    fi

    if [ $(grep -o $word $filename | wc -l )  -ge 3 ]; then
        echo "you are the best bro"
    fi

    shift 2
done
