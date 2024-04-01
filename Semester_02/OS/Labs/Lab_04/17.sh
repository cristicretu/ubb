cat ceva | sort | uniq -c | awk 'BEGIN{ans=0;} {ans+=$1-1;} END{print ans}'
