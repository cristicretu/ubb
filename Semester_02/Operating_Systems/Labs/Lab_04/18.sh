ls -l | awk 'NR > 1 && $5 < 100 {print $9}'
