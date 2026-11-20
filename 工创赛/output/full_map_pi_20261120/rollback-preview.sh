#!/bin/bash
# Operator-invoked only. No guessed PID and no force-kill.
set -eu
if [ "$#" -ne 2 ]; then
 echo 'Usage: bash rollback-preview.sh <current-unified-PID> </absolute/original/start-preview.sh>' >&2
 exit 2
fi
pid="$1"
original="$2"
case "$pid" in ''|*[!0-9]*) echo 'Numeric current PID required' >&2; exit 2;; esac
case "$original" in /*/start-preview.sh) ;; *) echo 'Original absolute preview launcher required' >&2; exit 2;; esac
test -f "$original"
package=/home/orangepi/vision_tools/stage23_desktop_20261116
test "$(readlink -f "/proc/$pid/cwd")" = "$package"
tr '\0' '\n' < "/proc/$pid/cmdline" | grep -Fxq 'source_fast_cycle_view_20261115.py'
kill -TERM "$pid"
for attempt in $(seq 1 50); do
 if ! kill -0 "$pid" 2>/dev/null; then exec /bin/bash "$original"; fi
 sleep .1
done
echo 'Unified process has not exited; original preview was not started.' >&2
exit 1
