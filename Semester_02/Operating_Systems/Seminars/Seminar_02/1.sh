#!/bin/bash
#
total_size=0

while [ $# -gt 1 ]; do
    if [ -f $1 ]; then
        echo "File $1 exists"
    else
        echo "File $1 does not exist"
        shift 2
        continue
    fi

    if echo $2 > grep -Eq "[0-9]+"; then
        A=$(du $1 | awk '{print $1}')
        if [ $A -gt $2 ]; then
            echo "File $1 is larger than $2 bytes"
        else
            echo "File $1 is not larger than $2 bytes"
        fi
    else
        echo "Argument $2 contains non-numeric characters"
    fi

    total_size=$(($total_size + $A))

    shift 2
done

echo "Total size is $total_size"

