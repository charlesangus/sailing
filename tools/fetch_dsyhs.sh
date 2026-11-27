#!/usr/bin/env bash
# Downloads the DSYHS hull geometries (doi:10.4121/21501330) and the series'
# model-scale hydrostatics table (doi:10.4121/21501375) from 4TU.ResearchData.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
raw="$root/data/dsyhs/raw"

GEOM_URL=https://data.4tu.nl/file/555b6bdc-0352-4287-bc1c-a47eb62ec901/b2cf3299-ccd9-4b7a-8214-113b23b8e338
GEOM_FILE=geometriesIGSmodelscale.zip
GEOM_SHA256=4d9794d4b12a672deee8b0c2afa36f8cf2ac857a3a86ecb646912e25240cb5f9

HYDRO_URL=https://data.4tu.nl/file/e198553e-0130-45d4-88cc-680d9f441b9b/8da67eeb-6d5a-4909-ac60-6fda527d774f
HYDRO_FILE=DSYHS_hydrostatics_modelscale.xlsx
HYDRO_SHA256=f064b12109e180f69b6da17cc824fd43c9b515e650631411eeea4d0017ba5983

fail() {
    echo "fetch_dsyhs: $1" >&2
    echo "  url      $2" >&2
    echo "  expected sha256 $3" >&2
    exit 1
}

# A file that fails verification is deleted so it is never unpacked or reused.
fetch() {
    local url="$1" sha="$2" out="$raw/$3"
    if [ -f "$out" ]; then
        if [ "$(sha256sum "$out" | cut -d' ' -f1)" = "$sha" ]; then
            return 0
        fi
        echo "fetch_dsyhs: cached $out failed checksum, re-downloading" >&2
        rm -f "$out"
    fi
    local part="$out.part"
    echo "fetch_dsyhs: downloading $3" >&2
    if ! curl --fail --location --silent --show-error --retry 2 -o "$part" "$url"; then
        rm -f "$part"
        fail "download failed" "$url" "$sha"
    fi
    local actual
    actual="$(sha256sum "$part" | cut -d' ' -f1)"
    if [ "$actual" != "$sha" ]; then
        rm -f "$part"
        echo "  actual   sha256 $actual" >&2
        fail "checksum mismatch" "$url" "$sha"
    fi
    mv "$part" "$out"
}

mkdir -p "$raw"
fetch "$GEOM_URL" "$GEOM_SHA256" "$GEOM_FILE"
fetch "$HYDRO_URL" "$HYDRO_SHA256" "$HYDRO_FILE"

if [ ! -f "$raw/IGS/.unpacked" ]; then
    rm -rf "${raw:?}/IGS"
    # zipfile.extractall drops absolute paths and ".." components from members.
    "$root/.venv/bin/python" -I -m zipfile -e "$raw/$GEOM_FILE" "$raw"
    touch "$raw/IGS/.unpacked"
fi

echo "fetch_dsyhs: ok ($(find "$raw/IGS" -name '*.igs' | wc -l) IGES files in $raw/IGS)"
