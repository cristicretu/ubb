#!/bin/bash
# Usage: ./5.sh filename

awk -F: '$1 ~ /88$/ {print $5}' "$1"

