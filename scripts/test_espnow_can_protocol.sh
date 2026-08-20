#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TEST_BIN="$(mktemp -t espnow-can-protocol-test.XXXXXX)"
trap 'rm -f "$TEST_BIN"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
  -I"$ROOT_DIR/main" \
  -I"$ROOT_DIR/main/inc" \
  "$ROOT_DIR/main/src/espnow_can_protocol.c" \
  "$ROOT_DIR/tests/espnow_can_protocol_test.c" \
  -o "$TEST_BIN"

"$TEST_BIN"
