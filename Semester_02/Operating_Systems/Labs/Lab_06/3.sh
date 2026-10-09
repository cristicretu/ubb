#!/bin/bash

# File to store emails
EMAIL_FILE="emails"

> "$EMAIL_FILE"

is_valid_username() {
    if [[ $1 =~ ^[a-zA-Z0-9]{3,30}$ ]]; then
        return 0
    else
        return 1
    fi
}

for username in "$@"; do
    if is_valid_username "$username"; then
        echo "${username}@ubbcluj.ro" >> "$EMAIL_FILE"
    else
        echo "Invalid username: $username" >&2
    fi
done

echo "Generated emails:"
cat "$EMAIL_FILE"

