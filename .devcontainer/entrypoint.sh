#!/usr/bin/env bash

source /opt/esp/idf/export.sh > /dev/null 2>&1 || true

exec "$@"
