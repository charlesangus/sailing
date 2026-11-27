#!/usr/bin/env bash
# Checks out Python-VPP (Marin Lauber, MIT licence) at a pinned commit into
# tools/vpp/vendor/Python-VPP and installs the pinned dependencies into .venv.
#
# PyPI python-vpp 1.0.0 is not used: it predates commit 3b4b0a3, which fixes
# swapped LVR/BTR axes in the residuary resistance lookup.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
vendor="$root/tools/vpp/vendor"
dest="$vendor/Python-VPP"

REPO_URL=https://github.com/marinlauber/Python-VPP
COMMIT=9b9fe8f3f63d902a1df207691109e42932d0a9bb

"$root/.venv/bin/python" -m pip install --quiet --disable-pip-version-check \
    -r "$root/tools/vpp/requirements.txt"

if [ -d "$dest/.git" ] && [ "$(git -C "$dest" rev-parse HEAD)" = "$COMMIT" ] \
    && [ -z "$(git -C "$dest" status --porcelain)" ]; then
    echo "fetch_pyvpp: ok ($dest at $COMMIT)"
    exit 0
fi

rm -rf "${root:?}/tools/vpp/vendor/Python-VPP"
mkdir -p "$dest"
git -C "$dest" init --quiet
git -C "$dest" remote add origin "$REPO_URL"
if ! git -C "$dest" fetch --quiet --depth 1 origin "$COMMIT"; then
    echo "fetch_pyvpp: fetch of $COMMIT from $REPO_URL failed" >&2
    exit 1
fi
git -C "$dest" -c advice.detachedHead=false checkout --quiet FETCH_HEAD
if [ "$(git -C "$dest" rev-parse HEAD)" != "$COMMIT" ]; then
    echo "fetch_pyvpp: checked out $(git -C "$dest" rev-parse HEAD), expected $COMMIT" >&2
    exit 1
fi
echo "fetch_pyvpp: ok ($dest at $COMMIT)"
