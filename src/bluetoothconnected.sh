#!/bin/bash
bluetoothctl << EOF > /dev/null
discoverable off
pairable off
agent NoInputNoOutput
default-agent
EOF
