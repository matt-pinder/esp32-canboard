#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# Default paths for the original project.
DEFAULT_SOURCE="$ROOT_DIR/viewer/viewer_esp.html"
DEFAULT_TARGET="$ROOT_DIR/spiffs/viewer_esp.min.html.gz"

usage() {
    echo "Usage:"
    echo "  $0 [--check|--apply]"
    echo "  $0 <source.html> <target.html.gz> [--check|--apply]"
    echo
    echo "Examples:"
    echo "  $0 --apply"
    echo "  $0 --check"
    echo "  $0 web_ui/index.html main/spiffs/index.min.html.gz --apply"
}

# ---------------------------------------------------------------------------
# Arguments
# ---------------------------------------------------------------------------

SOURCE="$DEFAULT_SOURCE"
TARGET="$DEFAULT_TARGET"
MODE="--check"

case "$#" in
    0)
        ;;
    1)
        case "$1" in
            --check|--apply)
                MODE="$1"
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                echo "Unknown argument: $1" >&2
                usage >&2
                exit 2
                ;;
        esac
        ;;
    2)
        SOURCE="$1"
        TARGET="$2"
        ;;
    3)
        SOURCE="$1"
        TARGET="$2"
        MODE="$3"

        if [ "$MODE" != "--check" ] && [ "$MODE" != "--apply" ]; then
            echo "Invalid mode: $MODE" >&2
            usage >&2
            exit 2
        fi
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

# Resolve relative paths from the project root rather than the current shell
# working directory.
case "$SOURCE" in
    /*) ;;
    *) SOURCE="$ROOT_DIR/$SOURCE" ;;
esac

case "$TARGET" in
    /*) ;;
    *) TARGET="$ROOT_DIR/$TARGET" ;;
esac

if [ ! -f "$SOURCE" ]; then
    echo "Source HTML does not exist: $SOURCE" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Temporary working files
# ---------------------------------------------------------------------------

WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/regenerate-viewer.XXXXXX")"
trap 'rm -rf "$WORK_DIR"' EXIT

EXTRACTED_JS="$WORK_DIR/source.js"
MINIFIED_JS="$WORK_DIR/source.min.js"
MINIFIED_HTML="$WORK_DIR/source.min.html"
GENERATED="$WORK_DIR/source.min.html.gz"

export SOURCE EXTRACTED_JS MINIFIED_JS MINIFIED_HTML

# ---------------------------------------------------------------------------
# Extract and validate the inline JavaScript
# ---------------------------------------------------------------------------

node <<'NODE'
const fs = require('fs');

const html = fs.readFileSync(process.env.SOURCE, 'utf8');

const scripts = html.match(/<script>/g) || [];
const scriptEnds = html.match(/<\/script>/g) || [];
const match = html.match(/<script>([\s\S]*?)<\/script>/);

if (scripts.length !== 1 || scriptEnds.length !== 1 || !match) {
    throw new Error(
        `Expected exactly one inline <script> block; found ` +
        `${scripts.length} starts and ${scriptEnds.length} ends`
    );
}

// Validate source JavaScript before attempting minification.
new Function(match[1]);

fs.writeFileSync(process.env.EXTRACTED_JS, match[1]);
NODE

# ---------------------------------------------------------------------------
# Minify JavaScript
# ---------------------------------------------------------------------------

terser "$EXTRACTED_JS" \
    --compress passes=3,toplevel=true,unsafe=true,unsafe_math=true,unsafe_arrows=true,pure_getters=true,booleans_as_integers=true,drop_console=false \
    --mangle toplevel=true \
    --ecma 2020 \
    --comments false \
    -o "$MINIFIED_JS"

# ---------------------------------------------------------------------------
# Put the minified JavaScript back into the HTML and validate the result
# ---------------------------------------------------------------------------

node <<'NODE'
const fs = require('fs');

const html = fs.readFileSync(process.env.SOURCE, 'utf8');
const minified = fs.readFileSync(process.env.MINIFIED_JS, 'utf8');

// Validate Terser's output independently.
new Function(minified);

const output = html.replace(
    /<script>[\s\S]*?<\/script>/,
    () => '<script>' + minified + '</script>'
);

const scripts = output.match(/<script>/g) || [];
const scriptEnds = output.match(/<\/script>/g) || [];
const outputScript = output.match(/<script>([\s\S]*?)<\/script>/);

if (scripts.length !== 1 || scriptEnds.length !== 1 || !outputScript) {
    throw new Error(
        `Generated HTML has ${scripts.length} script starts and ` +
        `${scriptEnds.length} script ends`
    );
}

// Validate the JavaScript as embedded in the final HTML.
new Function(outputScript[1]);

fs.writeFileSync(process.env.MINIFIED_HTML, output);
NODE

# ---------------------------------------------------------------------------
# Reproducible gzip
# ---------------------------------------------------------------------------

gzip -9 -n -c "$MINIFIED_HTML" > "$GENERATED"

# ---------------------------------------------------------------------------
# Check/apply
# ---------------------------------------------------------------------------

case "$MODE" in
    --check)
        if [ ! -f "$TARGET" ] || ! cmp -s "$GENERATED" "$TARGET"; then
            echo "Generated web asset is stale." >&2
            echo "Source: $SOURCE" >&2
            echo "Target: $TARGET" >&2
            echo "Run with --apply to regenerate it." >&2
            exit 1
        fi

        echo "Generated web asset is current."
        ;;

    --apply)
        mkdir -p "$(dirname "$TARGET")"

        if [ -f "$TARGET" ] && cmp -s "$GENERATED" "$TARGET"; then
            echo "Generated web asset is current."
        else
            cp "$GENERATED" "$TARGET"
            echo "Updated $TARGET"
        fi
        ;;
esac