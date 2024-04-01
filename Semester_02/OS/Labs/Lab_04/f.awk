NR > 1 && $1 ~ /[a-z0-9A-Z]{4}/ {
    if (!($1 in suma)) {
        suma[$1] = $2
    } else {
        suma[$1] += $2
    }
}

END {
    for (i in suma) {
        print i, suma[i]
    }
}
