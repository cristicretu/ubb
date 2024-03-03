#!/bin/bash
# Usage: ./3.sh filename

awk -F: '/^m/ && $3==int($3/7)*7  {print $5}' "$1"

