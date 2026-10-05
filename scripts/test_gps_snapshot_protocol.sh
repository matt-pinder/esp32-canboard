#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/gps_snapshot_protocol_test.XXXXXX")"
trap 'rm -f "$binary"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
  -I"$root/main/inc" -I"$root/main" \
  "$root/main/src/gps_snapshot_protocol.c" \
  "$root/tests/gps_snapshot_protocol_test.c" \
  -o "$binary"
"$binary"
