#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 <dir>"
    exit 1
fi

dir=$1

for x in $(find $dir); do
    if  file $x | grep -q "C" ; then
        echo $x
    fi
done
