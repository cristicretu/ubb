#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

if [ ! -f $1 ]; then
    echo "File not found!"
    exit 2
fi

res=""

for u in $(cat $1); do
    user_exists=$(grep "^$u:" /etc/passwd)
    if user_exists; then
        res="$u@scs.ubbcluj.ro,$result"
    fi
done

result=$(echo $res | sed 's/,$//')
echo $result
