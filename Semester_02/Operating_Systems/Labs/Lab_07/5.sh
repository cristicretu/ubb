#!/bin/bash


if [ $# -ne 1 ]; then
    echo "pls provide n <->"
    exit 1
fi

n=$1

total_lines=0

while read filename; do
    if [ ! -f $filename ]; then
        echo "not a file"
        continue
    fi

    lines=$(cat $filename | wc -l)
    total_lines=$((total_lines + lines))

    if [ $lines -lt $n ]; then
        echo $filename
    else
        echo "lines greater than $n"
        break
    fi
done

echo "total lines: $total_lines"


