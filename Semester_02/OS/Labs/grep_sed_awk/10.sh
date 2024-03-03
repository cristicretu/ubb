cat passwd.fake | sed -F 's/[a-zs-z 0-9A-Z/:-]//g' | sort -u
