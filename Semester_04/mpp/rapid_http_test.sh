#!/bin/bash

# Launch multiple curl requests in parallel to create exercises rapidly
NUM_REQUESTS=15  # Above the threshold of 10
MAX_PARALLEL=5   # Number of parallel requests

COOKIE="authjs.callback-url=http%3A%2F%2Flocalhost%3A3000%2Fauth%2Fsignin; authjs.session-token=eyJhbGciOiJkaXIiLCJlbmMiOiJBMjU2Q0JDLUhTNTEyIiwia2lkIjoiNDNUYV9OODBHbno0TTRhWHhZSkh2cWVvdmpxZ2lnS1o1a1g5SHMyTVlHVkx1dEl6MmxoUVBkZ0tIRGxxcmFOMFdRcVhmTnU2UUc4WjlkVEpJUzlhdncifQ..QxHT1MfJezrmOW28Wu693A.F6zhk6-LKpoC07R21tM7fRRlmMix5G8226CMJ30A44f9xEsDJ7My60jA_7RnuACMcGjrldPYBl9uN8B84oMy6b6EHoDQUVaO0vyTOk_9J5ZAfcFm4hZTh5jbAoFM07MuJUfgfvst7RNDyLo55pKYCfSkgqZl1wxam1Wf0WBnLS-DEEzNb2_0mow-NuSPhw2E-vg6sYeZIr7bJSbp7kQeQUphbsRsKUJWRzNWZSPKRHEs1vAJCnGj7ivLljaNeNZu.wMt1rm--ApoxKKHz3rufmMZSguOBu32lKfihFyP_2c8; __next_hmr_refresh_hash__=28; authjs.csrf-token=915909d37350acda4033f434e62bbc709c7a9bf62f4f4af3baac0bd4ffbd4ccf%7Cf945fe6ab27acb793b961fb3b6c660b4d2d02da032db330a680cda0456a2356c"

VIDEO_URL="http://localhost:3000/uploads/1746429384551_cfglsgm6.mp4"

# Clear monitored users first
echo "Clearing monitored users table..."
npx tsx clear_monitored_users.js

echo "Running RAPID exercise creation test..."
echo "Sending $NUM_REQUESTS requests with max $MAX_PARALLEL parallel connections"
echo "This should trigger monitoring (threshold: 10 in 10 seconds)"

# Create an array to store background process IDs
pids=()

# Function to send a single request
send_request() {
  local i=$1
  local name="RapidTest-Exercise-$i"
  
  curl 'http://localhost:3000/api/exercises' \
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
    -s > /dev/null &
  
  echo "Started request $i (PID: $!)"
  pids+=($!)
}

START_TIME=$(date +%s)

# Launch requests in parallel batches
for i in $(seq 1 $NUM_REQUESTS); do
  # If we have MAX_PARALLEL active processes, wait for one to finish
  if [ ${#pids[@]} -ge $MAX_PARALLEL ]; then
    wait -n  # Wait for any child process to exit
    # Remove completed PIDs
    for j in "${!pids[@]}"; do
      if ! kill -0 ${pids[$j]} 2>/dev/null; then
        unset pids[$j]
      fi
    done
    # Re-index array
    pids=("${pids[@]}")
  fi
  
  send_request $i
done

# Wait for all remaining processes to complete
wait

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo ""
echo "All $NUM_REQUESTS requests completed in $DURATION seconds"
echo "Wait a few seconds for the monitoring system to detect this activity"
echo "Then check the admin panel at http://localhost:3000/admin/monitoring to see if monitoring was triggered."

# Wait a bit and then check if the user is being monitored
echo ""
echo "Waiting 10 seconds for the monitoring system to run..."
sleep 10

# Check if any users are now monitored
echo "Checking monitored users..."
npx tsx -e "
const { PrismaClient } = require('@prisma/client');
const prisma = new PrismaClient();
async function check() {
  const count = await prisma.monitoredUser.count();
  const users = await prisma.monitoredUser.findMany({
    include: { user: { select: { email: true } } }
  });
  console.log(\`Found \${count} monitored users\`);
  for (const u of users) {
    console.log(\`- \${u.user.email}: \${u.reason}\`);
  }
  await prisma.$disconnect();
}
check();
" 