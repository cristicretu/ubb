#!/bin/bash

if [ $# -lt 1 ]; then
    echo "Usage: $0 <file>"
    exit 1
fi

find $1 -type f | grep -E "\.c$" | wc -l
