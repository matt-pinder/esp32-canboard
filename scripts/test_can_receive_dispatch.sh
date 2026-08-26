#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test_binary="$(mktemp /tmp/can-receive-dispatch-test.XXXXXX)"
trap 'rm -f "$test_binary"' EXIT

cc -std=c11 -Wall -Wextra -Werror \
  -I"$repo_dir/main" \
  -I"$repo_dir/main/inc" \
  "$repo_dir/main/src/can_receive_dispatch.c" \
  "$repo_dir/tests/can_receive_dispatch_test.c" \
  -o "$test_binary"
"$test_binary"
