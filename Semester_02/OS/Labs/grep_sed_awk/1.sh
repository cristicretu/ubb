#!/bin/bash
# Usage: ./1.sh filename

awk '$3 ~ /economica/ && $4 == "Sun" {print $1}' "$1" | sort -u

