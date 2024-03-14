#!/bin/bash


for A in $@; do
    if echo $A | grep -Eq "^[0-9]*[02468]$" && \
         (test -f $A || test -d $A); then
        echo "$A is an even number"
    elif  [ -f $A ]; then
        echo "File $A exists"
    elif [ -d $A ]; then
        echo "Directory $A exists"
    elif echo $A | grep -Eq "^[0-9]+$" ;then
        echo "$A is a number"
    else
        echo "$A does not exist"
    fi
done

