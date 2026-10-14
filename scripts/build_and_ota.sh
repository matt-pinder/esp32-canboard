#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"
firmware="$project_dir/build/esp32-canboard.bin"
spiffs_image="$project_dir/build/spiffs.bin"
ota_url="${ESP32_CANBOARD_OTA_URL:-http://192.168.4.1/api/ota}"
device_base="${ota_url%/api/ota}"
spiffs_url="${ESP32_CANBOARD_SPIFFS_URL:-$device_base/api/spiffs}"

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
  [[ -f "$firmware" && -f "$spiffs_image" ]] || return 0

  local input
  for input in \
    "$project_dir/CMakeLists.txt" \
    "$project_dir/sdkconfig" \
    "$project_dir/sdkconfig.defaults" \
    "$project_dir/partitions.csv" \
    "$project_dir/dependencies.lock" \
    "$project_dir/scripts/regenerate_viewer.sh"; do
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
status_json="$(curl --fail --silent --show-error \
  --connect-timeout 5 \
  --max-time 10 \
  "$device_base/api/status")"

if build_is_required; then
  echo "Build output is missing or stale; rebuilding before OTA."
  activate_idf_if_needed

  "$project_dir/scripts/regenerate_viewer.sh" \
    "$project_dir/web_ui/index.html" \
    "$project_dir/main/spiffs/index.min.html.gz" \
    --apply

  python "$IDF_PATH/tools/idf.py" -C "$project_dir" build
else
  echo "Using existing up-to-date build: $firmware"
fi

if [[ ! -f "$firmware" || ! -f "$spiffs_image" ]]; then
  echo "OTA artifacts were not generated: $firmware and $spiffs_image" >&2
  exit 1
fi

supports_spiffs_ota() {
  [[ "$1" =~ \"spiffs_ota\"[[:space:]]*:[[:space:]]*true ]]
}

upload_firmware() {
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
}

upload_spiffs() {
  local spiffs_sha
  spiffs_sha="$(shasum -a 256 "$spiffs_image" | awk '{print $1}')"
  echo "Uploading $(basename "$spiffs_image") to $spiffs_url"
  curl --fail-with-body --show-error --progress-bar \
    --http1.0 \
    --no-keepalive \
    --connect-timeout 5 \
    --max-time 180 \
    -H "Content-Type: application/octet-stream" \
    -H "X-Image-SHA256: $spiffs_sha" \
    -H "Connection: close" \
    -H "Expect:" \
    --data-binary "@$spiffs_image" \
    "$spiffs_url"
}

wait_for_spiffs_ota() {
  local attempt response
  for attempt in {1..30}; do
    if response="$(curl --fail --silent --show-error \
        --connect-timeout 1 --max-time 1 "$device_base/api/status" 2>/dev/null)" && \
       supports_spiffs_ota "$response"; then
      return 0
    fi
    sleep 1
  done
  echo "The updated firmware did not expose SPIFFS OTA within 60 seconds." >&2
  return 1
}

if supports_spiffs_ota "$status_json"; then
  # Update the mounted asset filesystem first, then reboot only once for the app.
  upload_spiffs
  upload_firmware
  echo "Application and SPIFFS OTA complete. The ESP32 is booting the new application."
else
  # Legacy firmware can install the new application endpoint through /api/ota.
  upload_firmware
  echo "Waiting for the new application to expose /api/spiffs..."
  wait_for_spiffs_ota
  upload_spiffs
  echo "Application and SPIFFS OTA bootstrap complete. No cable flash was required."
fi
exit 0
