#!/bin/bash

awk 'BEGIN {sum=0} {sum+=$2} END {print sum/NR}' "$1"
