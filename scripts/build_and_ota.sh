#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"
firmware="$project_dir/build/esp32-logger.bin"
ota_url="${ESP32_CANBOARD_OTA_URL:-http://192.168.4.1/api/ota}"
device_base="${ota_url%/api/ota}"

build_input_is_newer() {
  local input="$1"
  [[ -e "$input" && "$input" -nt "$firmware" ]]
}

build_tree_is_newer() {
  local tree="$1"
  [[ -d "$tree" ]] || return 1

  find "$tree" -type f \
    \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \
       -o -name 'CMakeLists.txt' -o -name '*.cmake' -o -name '*.yml' -o -name '*.yaml' \
       -o -name '*.html' -o -name '*.gz' \) \
    -newer "$firmware" -print -quit | grep -q .
}

build_is_required() {
  [[ -f "$firmware" ]] || return 0

  local input
  for input in \
    "$project_dir/CMakeLists.txt" \
    "$project_dir/sdkconfig" \
    "$project_dir/sdkconfig.defaults" \
    "$project_dir/partitions.csv" \
    "$project_dir/dependencies.lock" \
    "$project_dir/scripts/minify.sh"; do
    if build_input_is_newer "$input"; then
      return 0
    fi
  done

  if build_tree_is_newer "$project_dir/main" || \
     build_tree_is_newer "$project_dir/web_ui" || \
     build_tree_is_newer "$project_dir/components" || \
     build_tree_is_newer "$project_dir/managed_components"; then
    return 0
  fi

  return 1
}

activate_idf_if_needed() {
  local expected_idf="$HOME/.espressif/v6.0.2/esp-idf"

  if [[ "${IDF_PATH:-}" == "$expected_idf" && -x "${IDF_PYTHON_ENV_PATH:-}/bin/python" ]]; then
    return
  fi

  export IDF_TOOLS_PATH="$HOME/.espressif/tools"
  export IDF_PYTHON_ENV_PATH="$IDF_TOOLS_PATH/python/v6.0.2/venv"

  # ESP-IDF's export script probes optional shell variables while configuring
  # the toolchain, so nounset must be disabled while it is sourced.
  set +u
  source "$expected_idf/export.sh"
  set -u
}

echo "Checking ESP32 CanBoard at $device_base"
curl --fail --silent --show-error \
  --connect-timeout 5 \
  --max-time 10 \
  "$device_base/api/status" >/dev/null

if build_is_required; then
  echo "Build output is missing or stale; rebuilding before OTA."
  activate_idf_if_needed

  web_temp="$(mktemp -d)"
  trap 'rm -rf "$web_temp"' EXIT

  bash "$project_dir/scripts/minify.sh" "$project_dir/web_ui/index.html" "$web_temp/index.min.html"
  mv "$web_temp/index.min.html.gz" "$project_dir/main/spiffs/index.min.html.gz"
  python "$IDF_PATH/tools/idf.py" -C "$project_dir" build
else
  echo "Using existing up-to-date build: $firmware"
fi

if [[ ! -f "$firmware" ]]; then
  echo "Firmware binary was not generated: $firmware" >&2
  exit 1
fi

echo "Uploading $(basename "$firmware") to $ota_url"
curl --fail-with-body --show-error --progress-bar \
  --http1.0 \
  --no-keepalive \
  --connect-timeout 5 \
  --max-time 180 \
  -H "Content-Type: application/octet-stream" \
  -H "Connection: close" \
  -H "Expect:" \
  --data-binary "@$firmware" \
  "$ota_url"

echo "OTA upload complete. The ESP32 is booting the new application."
exit 0
