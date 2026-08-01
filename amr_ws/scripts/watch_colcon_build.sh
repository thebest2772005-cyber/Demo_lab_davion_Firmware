#!/usr/bin/env bash
# Watch the `src/` directory and run `colcon build` on changes (debounced).
set -euo pipefail

WATCH_DIR=${1:-src}
DEBOUNCE=${2:-2}
BUILD_CMD="colcon build --symlink-install"

if ! command -v inotifywait >/dev/null 2>&1; then
  echo "inotifywait not found. Install inotify-tools: sudo apt install inotify-tools"
  exit 1
fi

echo "Watching '$WATCH_DIR' for changes. Debounce=${DEBOUNCE}s"
last_build=0
while true; do
  # Block until an event occurs
  inotifywait -r -e modify,create,delete,move --exclude '(.swp$|~$|\.pyc$)' "$WATCH_DIR" >/dev/null 2>&1
  now=$(date +%s)
  elapsed=$((now - last_build))
  if [[ $elapsed -lt $DEBOUNCE ]]; then
    # Wait briefly to batch rapid events
    sleep $((DEBOUNCE - elapsed))
  fi

  echo "Change detected at $(date). Running: $BUILD_CMD"
  if $BUILD_CMD; then
    echo "Build succeeded at $(date)"
  else
    echo "Build failed at $(date)" >&2
  fi
  last_build=$(date +%s)
done
