#!/usr/bin/env bash
set -Eeuo pipefail

echo "This script deliberately does not edit the device tree automatically."
echo "Run: sudo orangepi-config"
echo "Select System -> Hardware -> uart6-m1 -> Save -> Reboot."
echo "After reboot verify /dev/ttyS* and confirm the selected node with a 3.3V loopback."
echo "Header pin 16 is TX, pin 18 is RX. Never connect 5V TTL."

