ps aux | grep -E "^([^ ]+[ ]+){10}(vi|nano|pico|emacs|joe)" | awk 'BEGIN{sum=0;} {sum+=$2;} {print sum}'

