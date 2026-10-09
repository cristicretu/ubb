#!/bin/bash

NUM_REQUESTS=100

COOKIE="authjs.callback-url=http%3A%2F%2Flocalhost%3A3000%2Fauth%2Fsignin; authjs.session-token=eyJhbGciOiJkaXIiLCJlbmMiOiJBMjU2Q0JDLUhTNTEyIiwia2lkIjoiNDNUYV9OODBHbno0TTRhWHhZSkh2cWVvdmpxZ2lnS1o1a1g5SHMyTVlHVkx1dEl6MmxoUVBkZ0tIRGxxcmFOMFdRcVhmTnU2UUc4WjlkVEpJUzlhdncifQ..QxHT1MfJezrmOW28Wu693A.F6zhk6-LKpoC07R21tM7fRRlmMix5G8226CMJ30A44f9xEsDJ7My60jA_7RnuACMcGjrldPYBl9uN8B84oMy6b6EHoDQUVaO0vyTOk_9J5ZAfcFm4hZTh5jbAoFM07MuJUfgfvst7RNDyLo55pKYCfSkgqZl1wxam1Wf0WBnLS-DEEzNb2_0mow-NuSPhw2E-vg6sYeZIr7bJSbp7kQeQUphbsRsKUJWRzNWZSPKRHEs1vAJCnGj7ivLljaNeNZu.wMt1rm--ApoxKKHz3rufmMZSguOBu32lKfihFyP_2c8; __next_hmr_refresh_hash__=28; authjs.csrf-token=915909d37350acda4033f434e62bbc709c7a9bf62f4f4af3baac0bd4ffbd4ccf%7Cf945fe6ab27acb793b961fb3b6c660b4d2d02da032db330a680cda0456a2356c"

echo "Starting to send $NUM_REQUESTS exercise creation requests..."

for i in $(seq 1 $NUM_REQUESTS); do
  NAME="Exercise-Test-$i"
  
  TIMESTAMP=$(date +"%T")
  
  response=$(curl 'http://localhost:3000/api/exercises' \
    -X 'POST' \
    -H 'Content-Type: application/json' \
    -H 'Accept: */*' \
    -H 'Sec-Fetch-Site: same-origin' \
    -H 'Accept-Language: en-US,en;q=0.9' \
    -H 'Accept-Encoding: gzip, deflate' \
    -H 'Sec-Fetch-Mode: cors' \
    -H 'Host: localhost:3000' \
    -H 'Origin: http://localhost:3000' \
    -H 'User-Agent: Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/18.2 Safari/605.1.15' \
    -H 'Referer: http://localhost:3000/' \
    -H 'Connection: keep-alive' \
    -H 'Sec-Fetch-Dest: empty' \
    -H "Cookie: $COOKIE" \
    --data-binary "{\"name\":\"$NAME\",\"videoUrl\":\"http://localhost:3000/uploads/test_video.mp4\",\"form\":\"good\",\"date\":\"2025-05-05T07:16:28.940Z\",\"duration\":10}" \
    -s)
  
  echo "[$TIMESTAMP] Request $i: $response"
  
  sleep 0.2
done

echo "All $NUM_REQUESTS requests sent. Check the admin panel to see if monitoring was triggered." 