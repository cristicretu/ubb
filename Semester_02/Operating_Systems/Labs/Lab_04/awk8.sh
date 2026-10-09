awk '{s=($3+$4+$5)/3; print $1" "$2" "s; if (s > 8) names[$1" "$2]; } END {for (np in names) {print np;}}' medie

