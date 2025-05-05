#!/bin/bash

NUM_REQUESTS=15
MAX_PARALLEL=5
VIDEO_URL="http://localhost:3000/uploads/1746429384551_cfglsgm6.mp4"

echo "Clearing monitored users table..."
npx tsx clear_monitored_users.js

COOKIE="authjs.callback-url=http%3A%2F%2Flocalhost%3A3000%2Fauth%2Fsignin; authjs.session-token=eyJhbGciOiJkaXIiLCJlbmMiOiJBMjU2Q0JDLUhTNTEyIiwia2lkIjoiNDNUYV9OODBHbno0TTRhWHhZSkh2cWVvdmpxZ2lnS1o1a1g5SHMyTVlHVkx1dEl6MmxoUVBkZ0tIRGxxcmFOMFdRcVhmTnU2UUc4WjlkVEpJUzlhdncifQ..QxHT1MfJezrmOW28Wu693A.F6zhk6-LKpoC07R21tM7fRRlmMix5G8226CMJ30A44f9xEsDJ7My60jA_7RnuACMcGjrldPYBl9uN8B84oMy6b6EHoDQUVaO0vyTOk_9J5ZAfcFm4hZTh5jbAoFM07MuJUfgfvst7RNDyLo55pKYCfSkgqZl1wxam1Wf0WBnLS-DEEzNb2_0mow-NuSPhw2E-vg6sYeZIr7bJSbp7kQeQUphbsRsKUJWRzNWZSPKRHEs1vAJCnGj7ivLljaNeNZu.wMt1rm--ApoxKKHz3rufmMZSguOBu32lKfihFyP_2c8; __next_hmr_refresh_hash__=28; authjs.csrf-token=915909d37350acda4033f434e62bbc709c7a9bf62f4f4af3baac0bd4ffbd4ccf%7Cf945fe6ab27acb793b961fb3b6c660b4d2d02da032db330a680cda0456a2356c"

echo "--------------------------------------------------------------"
echo "RAPID API REQUEST TEST - Sending $NUM_REQUESTS requests"
echo "Target: Will send requests as fast as possible to trigger monitoring"
echo "The monitoring system should detect users with > 10 actions in 10 seconds"
echo "--------------------------------------------------------------"

declare -a pids=()

send_request() {
  local i=$1
  local name="RapidTest-$i-$(date +%s.%N | cut -c1-13)"
  
  curl 'http://localhost:3000/api/exercises' \
    -X 'POST' \
    -H 'Content-Type: application/json' \
    -H 'Accept: */*' \
    -H 'Origin: http://localhost:3000' \
    -H 'User-Agent: Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7)' \
    -H 'Referer: http://localhost:3000/' \
    -H "Cookie: $COOKIE" \
    --data-binary "{\"name\":\"$name\",\"videoUrl\":\"$VIDEO_URL\",\"form\":\"good\",\"date\":\"2025-05-05T07:16:28.940Z\",\"duration\":10}" \
    --silent \
    --output /dev/null &
  
  pids+=($!)
  echo "→ Started request $i (PID: $!)"
}

echo "Sending requests..."
START_TIME=$(date +%s)

for i in $(seq 1 $NUM_REQUESTS); do
  send_request $i
  
  sleep 0.1
done

echo "Waiting for all requests to finish..."
wait

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo "--------------------------------------------------------------"
echo "✅ All $NUM_REQUESTS requests completed in $DURATION seconds"
echo "Wait for 15 seconds for the monitoring system to run checks..."
echo "--------------------------------------------------------------"

sleep 15

echo "Checking if monitoring was triggered..."
CHECK_CMD="npx tsx -e \"
const { PrismaClient } = require('@prisma/client');
const prisma = new PrismaClient();
async function check() {
  const monitored = await prisma.monitoredUser.findMany({
    include: { user: { select: { email: true } } }
  });
  
  console.log('\\nMonitored Users:');
  if (monitored.length === 0) {
    console.log('  None - monitoring was NOT triggered!');
  } else {
    monitored.forEach(m => {
      console.log(\`  - \${m.user.email || m.userId}: \${m.reason}\`);
    });
  }
  
  await prisma.\$disconnect();
}
check();
\""

eval "$CHECK_CMD"

echo "--------------------------------------------------------------"
echo "Test completed. If a user is shown above, monitoring is working!"
echo "--------------------------------------------------------------" 