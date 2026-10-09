#!/bin/bash
# Usage: ./2.sh filename

cat "$1" | head -n 1689 | awk '$7 ~ /23:[0-9]+/ {print $1}' | sort -u

