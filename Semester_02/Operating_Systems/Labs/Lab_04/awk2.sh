awk 'BEGIN {pare=0; impare=0;} NR%2==0 {pare += NF;} NR%2==1 {impare += NF;} END{print pare; print (impare/(NR-1))} ' ceva
