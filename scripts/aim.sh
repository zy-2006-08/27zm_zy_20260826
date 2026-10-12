#!/usr/bin/env bash
# ============================================================================
#  Manage the background self-aim program started by Alt+N (scripts/run_current.sh).
# ----------------------------------------------------------------------------
#  When launched over SSH, the program runs inside a screen session named "aim",
#  so unplugging the cable / WiFi dropping does NOT kill it.
#
#    bash scripts/aim.sh status   # is it running? last log lines + fps
#    bash scripts/aim.sh attach   # watch live output again (detach: Ctrl+A then D)
#    bash scripts/aim.sh log      # follow the log file (Ctrl+C only stops viewing)
#    bash scripts/aim.sh stop     # stop the program
# ============================================================================
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="$ROOT/build/logs/aim_screen.log"

running() { screen -ls 2>/dev/null | grep -q '[0-9]\.aim[[:space:]]'; }

case "${1:-status}" in
    attach)
        running || { echo "aim is not running"; exit 1; }
        exec screen -r aim ;;
    log)
        exec tail -n 50 -f "$LOG" ;;
    stop)
        if running; then
            screen -S aim -X stuff $'\003'   # Ctrl+C: let it send the "stop control" packet to the C board
            for _ in $(seq 1 10); do sleep 0.3; running || break; done
            running && screen -S aim -X quit
            echo "aim stopped"
        else
            echo "aim is not running"
        fi ;;
    status|*)
        if running; then echo "aim: RUNNING"; else echo "aim: not running"; fi
        if [ -f "$LOG" ]; then
            grep -a "fps::" "$LOG" | tail -n 100 | awk '{s+=$NF;n++} END{if(n) printf "recent avg fps: %.1f\n", s/n}'
            tail -n 5 "$LOG" | cut -c1-160
        fi ;;
esac
