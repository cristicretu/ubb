#!/bin/bash

# calculates the sum and average of all the pid's in the system

sum=0
for pid in $(ps -e | tail -n +2 | awk '{print $2}'); do
    sum=$((sum + pid))
done

echo "Sum: $sum"
echo "Average: $((sum / $(ps -e | wc -l)))"
