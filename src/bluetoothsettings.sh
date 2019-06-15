#!/bin/bash
bluetoothctl << EOF > /dev/null
discoverable on
pairable on
agent NoInputNoOutput
default-agent
EOF