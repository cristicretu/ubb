#!/bin/bash

if [ $# -eq 0 ]; then
    echo "comon bro"
    exit 1
fi

str=$1

shift 1

while [ $# -ne 0 ]; do
    if [ -f $1 ]; then
        if cat $1 |  grep -q "$str"; then
            echo "$str is in file $1"
        else
            echo "$str IS NOT IN THE FILE $1"
        fi
    elif [ -d $1 ]; then
        if find $1 -name $str | grep -q "$str"; then
            echo "file with name $str found in directory $1"
        fi
    fi
    shift 1
done
