#!/bin/bash

ls -l | awk 'NR > 1 &&  $1 ~ /^-/ {print $9 "-" $1}' | awk -F- '{print $1" " $3}'
