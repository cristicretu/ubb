#!/bin/bash

NUM_REQUESTS=10000    
DELAY=0.1             
BATCH_SIZE=100        
VIDEO_URL="http://localhost:3000/uploads/1746429384551_cfglsgm6.mp4"  

while [[ $# -gt 0 ]]; do
  case $1 in
    -n|--num)
      NUM_REQUESTS="$2"
      shift 2
      ;;
    -d|--delay)
      DELAY="$2"
      shift 2
      ;;
    -b|--batch)
      BATCH_SIZE="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [-n|--num NUM_REQUESTS] [-d|--delay DELAY_SECONDS] [-b|--batch BATCH_SIZE]"
      exit 1
      ;;
  esac
done

COOKIE="authjs.callback-url=http%3A%2F%2Flocalhost%3A3000%2Fauth%2Fsignin; authjs.session-token=eyJhbGciOiJkaXIiLCJlbmMiOiJBMjU2Q0JDLUhTNTEyIiwia2lkIjoiNDNUYV9OODBHbno0TTRhWHhZSkh2cWVvdmpxZ2lnS1o1a1g5SHMyTVlHVkx1dEl6MmxoUVBkZ0tIRGxxcmFOMFdRcVhmTnU2UUc4WjlkVEpJUzlhdncifQ..QxHT1MfJezrmOW28Wu693A.F6zhk6-LKpoC07R21tM7fRRlmMix5G8226CMJ30A44f9xEsDJ7My60jA_7RnuACMcGjrldPYBl9uN8B84oMy6b6EHoDQUVaO0vyTOk_9J5ZAfcFm4hZTh5jbAoFM07MuJUfgfvst7RNDyLo55pKYCfSkgqZl1wxam1Wf0WBnLS-DEEzNb2_0mow-NuSPhw2E-vg6sYeZIr7bJSbp7kQeQUphbsRsKUJWRzNWZSPKRHEs1vAJCnGj7ivLljaNeNZu.wMt1rm--ApoxKKHz3rufmMZSguOBu32lKfihFyP_2c8; __next_hmr_refresh_hash__=28; authjs.csrf-token=915909d37350acda4033f434e62bbc709c7a9bf62f4f4af3baac0bd4ffbd4ccf%7Cf945fe6ab27acb793b961fb3b6c660b4d2d02da032db330a680cda0456a2356c"

START_TIME=$(date +%s)

echo "Starting stress test with $NUM_REQUESTS exercise creation requests..."
echo "- Delay between requests: ${DELAY}s"
echo "- Status updates every $BATCH_SIZE requests"
echo "Press Ctrl+C to stop the test at any time"
echo "----------------------------------------------"

SUCCESS_COUNT=0
FAIL_COUNT=0

for i in $(seq 1 $NUM_REQUESTS); do
  NAME="StressTest-Exercise-$i"
  
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
    --data-binary "{\"name\":\"$NAME\",\"videoUrl\":\"$VIDEO_URL\",\"form\":\"good\",\"date\":\"2025-05-05T07:16:28.940Z\",\"duration\":10}" \
    --write-out '%{http_code}' \
    --silent \
    --output /dev/null)
  
  if [[ "$response" == "2"* ]]; then
    ((SUCCESS_COUNT++))
  else
    ((FAIL_COUNT++))
  fi
  
  if (( i % BATCH_SIZE == 0 )); then
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - START_TIME))
    RATE=$(echo "scale=2; $i / $ELAPSED" | bc)
    
    echo "[$(date +"%T")] Progress: $i/$NUM_REQUESTS requests sent (${RATE}/sec) - Success: $SUCCESS_COUNT, Failed: $FAIL_COUNT"
  fi
  
  sleep $DELAY
done

END_TIME=$(date +%s)
TOTAL_TIME=$((END_TIME - START_TIME))
TOTAL_REQUESTS=$((SUCCESS_COUNT + FAIL_COUNT))
RATE=$(echo "scale=2; $TOTAL_REQUESTS / $TOTAL_TIME" | bc)

echo ""
echo "Test completed in ${TOTAL_TIME} seconds"
echo "Total requests: $TOTAL_REQUESTS (${RATE}/sec)"
echo "Successful: $SUCCESS_COUNT"
echo "Failed: $FAIL_COUNT"
echo ""
echo "Check the admin panel at http://localhost:3000/admin/monitoring to see if monitoring was triggered." 