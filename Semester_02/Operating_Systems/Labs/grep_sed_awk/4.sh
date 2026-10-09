#!/bin/bash
# Usage: ./4.sh filename

awk '$1=="root" {print $6}' "$1" | sort -u

