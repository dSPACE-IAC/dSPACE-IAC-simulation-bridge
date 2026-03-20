#!/bin/sh

echo "[INFO] Checking for privileged mode..."

# Try to create interface to test for privileges
ip link add dev $CAN_INTERFACE type vcan 2>/dev/null

if [ $? -eq 0 ]; then
    echo "[INFO] Privileged mode detected. Setting up vcan interfaces..."
    ip link set up $CAN_INTERFACE
else
    echo "[WARN] Not running in privileged mode or vcan module not available. Skipping automatic can interface setup. Setup of the interfaces needs to be done on the host and container needs to be started in host network."
fi
