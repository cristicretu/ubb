ls -l | grep -E '^-(r..){3}' | awk '{print $9}'
