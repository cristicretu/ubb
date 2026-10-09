awk -F: '{match($6, /\/gr[0-9]+\//); print $5 " - " $1 " - " substr($6, RSTART+3, RLENGTH-4)}' passwd.fake
