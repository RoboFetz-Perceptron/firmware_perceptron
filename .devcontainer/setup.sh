#!/usr/bin/env bash
set -euo pipefail

echo "[setup] Initializing RC Car firmware workspace..."

source /opt/esp/idf/export.sh

cd /workspaces/firmware

# Install Python requirements if present
if [ -f requirements.txt ]; then
    pip install --no-cache-dir -r requirements.txt
fi

echo "[setup] Done. Build with: idf.py build"
