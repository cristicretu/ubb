#!/bin/bash

if [ $# -lt 3 ]; then
    echo "at least provde a single triplet bro"
    exit 1
fi

while [ $# -gt 2 ]; do
     dir=$1
     n=$2
     m=$3

     if [ $n -gt $m ]; then
         echo "n should be less than m dumboo"
         shift 3
         continue
     fi

     if [ -d $dir ]; then
         no_of_files=$(find $dir -type f | wc -l)
         if [ $no_of_files -ge $n ] && [ $no_of_files -le $m ]; then
             echo "$dir has between $n and $m files"
         fi
     else
         echo "$dir doesnt exist"
     fi


     shift 3
 done
