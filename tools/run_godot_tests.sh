#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

godot="$repo_root/tools/godot/godot"
cli="$repo_root/build/dev/cli/sailsim"

if [[ ! -x "$cli" ]]; then
    echo "missing $cli: run 'cmake --build --preset dev' first" >&2
    exit 1
fi

if [[ -z "${SAIL_EXPECTED_VERSION:-}" ]]; then
    SAIL_EXPECTED_VERSION="$("$cli" --version)"
fi
export SAIL_EXPECTED_VERSION

# --script runs only see GDExtension classes listed in .godot/extension_list.cfg. Without that file,
# --import discovers and loads the extension mid-scan, and Godot 4.7.2 then segfaults on shutdown
# (exit 134), so seed it to make the extension load at startup as it does in an existing checkout.
shopt -s nullglob
extensions=(game/*.gdextension)
shopt -u nullglob
if [[ ! -f game/.godot/extension_list.cfg && ${#extensions[@]} -gt 0 ]]; then
    mkdir -p game/.godot
    printf 'res://%s\n' "${extensions[@]#game/}" >game/.godot/extension_list.cfg
fi

import_status=0
import_output="$("$godot" --headless --path game --import 2>&1)" || import_status=$?
# A GDExtension that fails to load only logs an ERROR; --import still exits 0.
if [[ $import_status -ne 0 ]] ||
    grep -qE '^ERROR: .*([Ee]xtension|dynamic library)' <<<"$import_output"; then
    printf '%s\n' "$import_output" >&2
    echo "godot --import failed (exit $import_status)" >&2
    exit 1
fi

failed=()
shopt -s nullglob
tests=(game/tests/*.gd)
if [[ ${#tests[@]} -eq 0 ]]; then
    echo "no tests found in game/tests" >&2
    exit 1
fi

for test_path in "${tests[@]}"; do
    name="$(basename "$test_path")"
    echo "== $name"
    status=0
    output="$("$godot" --headless --path game --script "res://tests/$name" 2>&1)" || status=$?
    printf '%s\n' "$output"

    # A parse error in --script mode can still exit 0, so also require the PASS marker.
    if [[ $status -ne 0 ]]; then
        failed+=("$name (exit $status)")
    elif grep -qE 'SCRIPT ERROR|Parse Error' <<<"$output"; then
        failed+=("$name (script error in output)")
    # push_error() from the extension never fails a script, so look for it in the output. Tests
    # of the error paths are named *_errors.gd and expected to print errors.
    elif [[ $name != *_errors.gd ]] && grep -qE '^ERROR:' <<<"$output"; then
        failed+=("$name (error in output)")
    elif ! grep -qx 'PASS' <<<"$output"; then
        failed+=("$name (no PASS marker)")
    fi
done

if [[ ${#failed[@]} -gt 0 ]]; then
    printf 'FAILED: %s\n' "${failed[@]}" >&2
    exit 1
fi
echo "all godot tests passed"
