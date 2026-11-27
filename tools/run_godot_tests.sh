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

# Creates .godot/extension_list.cfg, without which --script runs cannot see GDExtension classes.
if ! import_output="$("$godot" --headless --path game --import 2>&1)"; then
    printf '%s\n' "$import_output" >&2
    echo "godot --import failed" >&2
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
    elif ! grep -qx 'PASS' <<<"$output"; then
        failed+=("$name (no PASS marker)")
    fi
done

if [[ ${#failed[@]} -gt 0 ]]; then
    printf 'FAILED: %s\n' "${failed[@]}" >&2
    exit 1
fi
echo "all godot tests passed"
