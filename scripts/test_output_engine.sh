#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bin="$(mktemp "${TMPDIR:-/tmp}/output_engine_test.XXXXXX")"
trap 'rm -f "$bin"' EXIT

cc \
  -D_GNU_SOURCE \
  -std=c11 \
  -Wall -Wextra -Werror \
  -include "$root/tests/stubs/test_compat.h" \
  -I"$root/tests/stubs" \
  -I"$root/main" \
  -I"$root/main/inc" \
  "$root/main/src/relay_rule_engine.c" \
  "$root/tests/stubs/esp_partition_stub.c" \
  "$root/tests/output_engine_test.c" \
  -lm \
  -o "$bin"

"$bin"
