cat passwd.fake | awk -F: '{print $1}' | sed 'y/0123456789/1234567890/'
