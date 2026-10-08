#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"

"$project_dir/scripts/regenerate_web.sh" \
    "$project_dir/web_ui/index.html" \
    "$project_dir/main/spiffs/index.min.html.gz" \
    --apply