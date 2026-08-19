#!/usr/bin/env bash
set -euo pipefail

SRC="${1:?Usage: $0 <input.html> [output.html]}"
OUT="${2:-${SRC%.html}.min.html}"

TMP_JS="$(mktemp)"
TMP_MIN="$(mktemp)"
TMP_GZ="$(mktemp)"
trap 'rm -f "$TMP_JS" "$TMP_MIN" "$TMP_GZ"' EXIT

export SRC OUT TMP_JS TMP_MIN

node <<'NODE'
const fs = require('fs');

const html = fs.readFileSync(process.env.SRC, 'utf8');
const m = html.match(/<script>([\s\S]*?)<\/script>/);

if (!m) throw new Error('No inline script found');

new Function(m[1]);
fs.writeFileSync(process.env.TMP_JS, m[1]);
NODE

terser "$TMP_JS" \
  --compress passes=3,toplevel=true,unsafe=true,unsafe_math=true,unsafe_arrows=true,pure_getters=true,drop_console=false \
  --mangle toplevel=true \
  --ecma 2020 \
  --comments false \
  -o "$TMP_MIN"

node <<'NODE'
const fs = require('fs');

const html = fs.readFileSync(process.env.SRC, 'utf8');
const min = fs.readFileSync(process.env.TMP_MIN, 'utf8');

new Function(min);

fs.writeFileSync(
  process.env.OUT,
  html.replace(
    /<script>[\s\S]*?<\/script>/,
    '<script>' + min + '</script>'
  )
);
NODE


gzip -9 -n -c "$OUT" > "$TMP_GZ"
mv "$TMP_GZ" "$OUT.gz"

echo "Wrote $OUT.gz"
