#!/bin/bash

echo "Running monitoring system test..."
echo "This will create activity logs directly in the database and check if monitoring is triggered."

# Run the test script using project's Node.js environment
npx tsx test_monitor.js

echo "Test complete." 