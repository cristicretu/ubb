awk 'BEGIN{v=0; c=0} $0 ~ /[aeiouAEIOU]$/ {v++;} $0 ~ /[^aeiouAEIOU]/ && $0 ~ /[a-zA-Z]$/ {c++;} END {print v; print c;}' ceva
