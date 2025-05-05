#!/bin/bash

# Run the script using the Node.js environment of the project
echo "Clearing monitored users table..."
npx tsx clear_monitored_users.js

# Confirm completion
echo "Done. You can now run your test scripts." 