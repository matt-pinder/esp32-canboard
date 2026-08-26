#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TEST_BIN="$(mktemp -t mk60-emulator-protocol-test.XXXXXX)"
trap 'rm -f "$TEST_BIN"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
  -I"$ROOT_DIR/main" \
  -I"$ROOT_DIR/main/inc" \
  "$ROOT_DIR/main/src/mk60_emulator_protocol.c" \
  "$ROOT_DIR/tests/mk60_emulator_protocol_test.c" \
  -o "$TEST_BIN"

"$TEST_BIN"
