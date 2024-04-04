#!/bin/bash

while true; do
    read -r input

    if [[ $input == "stop" ]]; then
        break
    fi

    if [[ -f $input && $(file --mime "$input") =~ text/plain ]]; then
        head -n 3 "$input"
    else
        echo "error"
    fi
done

