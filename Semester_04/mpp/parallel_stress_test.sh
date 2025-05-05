#!/bin/bash

# Configuration variables
NUM_REQUESTS=10000     # Total number of requests
PARALLEL_JOBS=10       # Number of parallel jobs
BATCH_SIZE=100         # Print status update after this many requests
VIDEO_URL="http://localhost:3000/uploads/1746429384551_cfglsgm6.mp4"  # Video URL from original request

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    -n|--num)
      NUM_REQUESTS="$2"
      shift 2
      ;;
    -j|--jobs)
      PARALLEL_JOBS="$2"
      shift 2
      ;;
    -b|--batch)
      BATCH_SIZE="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [-n|--num NUM_REQUESTS] [-j|--jobs PARALLEL_JOBS] [-b|--batch BATCH_SIZE]"
      exit 1
      ;;
  esac
done

# Check if GNU Parallel is installed
if ! command -v parallel &> /dev/null; then
    echo "GNU Parallel is not installed. Please install it and try again."
    echo "On macOS: brew install parallel"
    echo "On Ubuntu/Debian: apt-get install parallel"
    exit 1
fi

# Get the cookie from the user's request
COOKIE="authjs.callback-url=http%3A%2F%2Flocalhost%3A3000%2Fauth%2Fsignin; authjs.session-token=eyJhbGciOiJkaXIiLCJlbmMiOiJBMjU2Q0JDLUhTNTEyIiwia2lkIjoiNDNUYV9OODBHbno0TTRhWHhZSkh2cWVvdmpxZ2lnS1o1a1g5SHMyTVlHVkx1dEl6MmxoUVBkZ0tIRGxxcmFOMFdRcVhmTnU2UUc4WjlkVEpJUzlhdncifQ..QxHT1MfJezrmOW28Wu693A.F6zhk6-LKpoC07R21tM7fRRlmMix5G8226CMJ30A44f9xEsDJ7My60jA_7RnuACMcGjrldPYBl9uN8B84oMy6b6EHoDQUVaO0vyTOk_9J5ZAfcFm4hZTh5jbAoFM07MuJUfgfvst7RNDyLo55pKYCfSkgqZl1wxam1Wf0WBnLS-DEEzNb2_0mow-NuSPhw2E-vg6sYeZIr7bJSbp7kQeQUphbsRsKUJWRzNWZSPKRHEs1vAJCnGj7ivLljaNeNZu.wMt1rm--ApoxKKHz3rufmMZSguOBu32lKfihFyP_2c8; __next_hmr_refresh_hash__=28; authjs.csrf-token=915909d37350acda4033f434e62bbc709c7a9bf62f4f4af3baac0bd4ffbd4ccf%7Cf945fe6ab27acb793b961fb3b6c660b4d2d02da032db330a680cda0456a2356c"

# Create a temporary directory for logs
TEMP_DIR=$(mktemp -d)
LOG_FILE="$TEMP_DIR/stress_test_results.log"

# Function to send a single request
send_request() {
  local i=$1
  local name="ParallelTest-Exercise-$i"
  
  # Send the request and get the status code
  local status=$(curl 'http://localhost:3000/api/exercises' \
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
    --data-binary "{\"name\":\"$name\",\"videoUrl\":\"$VIDEO_URL\",\"form\":\"good\",\"date\":\"2025-05-05T07:16:28.940Z\",\"duration\":10}" \
    --write-out '%{http_code}' \
    --silent \
    --output /dev/null)
  
  # Log the result
  echo "$i,$status" >> "$LOG_FILE"
  
  # Optional: Print progress for larger batches
  if (( i % BATCH_SIZE == 0 )); then
    echo "Completed $i requests"
  fi
}

echo "Starting parallel stress test with $NUM_REQUESTS exercise creation requests..."
echo "- Using $PARALLEL_JOBS parallel jobs"
echo "- Logging results to $LOG_FILE"
echo "----------------------------------------------"

# Start time for total duration calculation
START_TIME=$(date +%s)

# Export the functions and variables needed by parallel
export -f send_request
export COOKIE VIDEO_URL LOG_FILE BATCH_SIZE

# Run the requests in parallel
seq 1 $NUM_REQUESTS | parallel -j $PARALLEL_JOBS --progress send_request

# Calculate and display summary
END_TIME=$(date +%s)
TOTAL_TIME=$((END_TIME - START_TIME))
TOTAL_REQUESTS=$(wc -l < "$LOG_FILE")
SUCCESS_COUNT=$(grep -c ",2[0-9][0-9]$" "$LOG_FILE")
FAIL_COUNT=$((TOTAL_REQUESTS - SUCCESS_COUNT))
RATE=$(echo "scale=2; $TOTAL_REQUESTS / $TOTAL_TIME" | bc)

echo ""
echo "Test completed in ${TOTAL_TIME} seconds"
echo "Total requests: $TOTAL_REQUESTS (${RATE}/sec)"
echo "Successful: $SUCCESS_COUNT"
echo "Failed: $FAIL_COUNT"
echo ""
echo "Check the admin panel at http://localhost:3000/admin/monitoring to see if monitoring was triggered."

# Cleanup
rm -rf "$TEMP_DIR" 