#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
lock="${GODOT_LOCK:-$root/tools/godot.lock}"
dl="${GODOT_DIR:-$root/tools/godot}"

CMAKE_VERSION=3.31.6
NINJA_VERSION=1.13.0

# shellcheck disable=SC1090
source "$lock"

if [ ! -x "$root/.venv/bin/python" ]; then
    python3 -m venv "$root/.venv"
fi
"$root/.venv/bin/python" -m pip install --quiet --disable-pip-version-check \
    "cmake==$CMAKE_VERSION" "ninja==$NINJA_VERSION"

# Verify before anything is unpacked; a bad file is deleted so it is never reused.
fetch() {
    local url="$1" sha="$2" out="$3"
    if [ -f "$out" ]; then
        return 0
    fi
    local part="$out.part"
    if ! curl --fail --location --silent --show-error -o "$part" "$url"; then
        rm -f "$part"
        echo "bootstrap: download failed: $url (expected sha512 $sha)" >&2
        exit 1
    fi
    local actual
    actual="$(sha512sum "$part" | cut -d' ' -f1)"
    if [ "$actual" != "$sha" ]; then
        rm -f "$part"
        echo "bootstrap: checksum mismatch for $url" >&2
        echo "  expected $sha" >&2
        echo "  actual   $actual" >&2
        exit 1
    fi
    mv "$part" "$out"
}

mkdir -p "$dl"
editor_zip="$dl/$(basename "$GODOT_EDITOR_URL")"
templates_tpz="$dl/$(basename "$GODOT_TEMPLATES_URL")"
editor_bin="$dl/Godot_v${GODOT_VERSION}_linux.x86_64"

fetch "$GODOT_EDITOR_URL" "$GODOT_EDITOR_SHA512" "$editor_zip"
fetch "$GODOT_TEMPLATES_URL" "$GODOT_TEMPLATES_SHA512" "$templates_tpz"

if [ ! -x "$editor_bin" ]; then
    "$root/.venv/bin/python" -I -m zipfile -e "$editor_zip" "$dl"
    chmod +x "$editor_bin"
fi
ln -sfn "$(basename "$editor_bin")" "$dl/godot"

tpl_dir="$dl/templates/${GODOT_VERSION/-/.}"
if [ ! -d "$tpl_dir" ]; then
    tmp="$dl/templates.tmp"
    rm -rf "$tmp"
    mkdir -p "$tmp"
    "$root/.venv/bin/python" -I -m zipfile -e "$templates_tpz" "$tmp"
    mkdir -p "$dl/templates"
    mv "$tmp/templates" "$tpl_dir"
    rm -rf "$tmp"
fi

echo "bootstrap: ok (godot $GODOT_VERSION, godot-cpp $GODOT_CPP_REF)"
