#!/usr/bin/env bash
set -euo pipefail

# Resolve the repository root dynamically based on where this script lives
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_NAME="t3_pnr"
LOG_DIR="$SCRIPT_DIR/pnr_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/harden_$(date +%Y%m%d_%H%M%S).log"

# Automatically wrap execution inside a persistent tmux session so SSH disconnects don't kill PnR
if [ -z "${TMUX:-}" ]; then
    exec tmux new -As "$SESSION_NAME" "bash '$SCRIPT_DIR/$(basename "$0")' --in-tmux"
fi

cd "$SCRIPT_DIR"

# Activate Python virtual environment if present
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
elif [ -f "$HOME/venv/bin/activate" ]; then
    source "$HOME/venv/bin/activate"
fi

echo "==========================================" | tee -a "$LOG_FILE"
echo "[$(date)] Starting GF180MCU PnR in tmux ($SESSION_NAME)" | tee -a "$LOG_FILE"
echo "Working Dir: $SCRIPT_DIR" | tee -a "$LOG_FILE"
echo "Logging to:  $LOG_FILE" | tee -a "$LOG_FILE"
echo "==========================================" | tee -a "$LOG_FILE"

# Validate src/config.json syntax before starting
python3 -m json.tool src/config.json > /dev/null

# Run Tiny Tapeout / LibreLane hardening on gf180mcuD
./tt/tt_tool.py --pdk gf180mcuD --harden 2>&1 | tee -a "$LOG_FILE"

echo "==========================================" | tee -a "$LOG_FILE"
echo "[$(date)] PnR FINISHED SUCCESSFULLY" | tee -a "$LOG_FILE"
echo "==========================================" | tee -a "$LOG_FILE"