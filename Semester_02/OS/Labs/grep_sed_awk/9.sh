cat passwd.fake | sed -E 's/[a-zA-Z0-9 ]//g' | sort -u
