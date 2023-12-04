#!/bin/sh

ip link set can0 up type can bitrate 1000000
sleep 5
${PWD}/build/examples/csp_server -a 3 -c can0
