 #!/bin/bash

if [ $# -eq 0 ]; then
    echo "provide a username bro"
    exit 1
fi

while [ $# -gt 0 ]; do
    user_exists=$(grep -c "^$1:" /etc/passwd)

    if [ $user_exists -eq 1 ]; then
        valid_users+=( $1 )
    fi
    shift
done

while true; do
    for usr in ${valid_users[@]}; do
         ans=$(ps -ef | grep -c "$usr")
         echo "$usr has $ans running processes"
    done
    sleep 1
done
