#!/bin/sh
# Regenerates the LEO Ring Road scenarios of this directory with dtn-tvg-util.
#
# Usage: ./generate.sh [python]
#   python: interpreter with `dtn-tvg-util[ring_road]` installed (default: python3)
#
# TLEs are read from cubesat_tle.txt (CelesTrak "cubesat" group snapshot), not downloaded,
# so the output only depends on this file, the seed and the start time.
# Existing files are kept: delete them to regenerate.
set -eu

PY="${1:-python3}"
cd "$(dirname "$0")"

TLE=cubesat_tle.txt
START=1790860000 # 2026-10-01 13:06:40 UTC
SEED=1

# name sats gs rr islrange
while read -r name sats gs rr islrange; do
    if [ -e "$name.json" ]; then
        echo "== $name: $name.json exists, skipping"
        continue
    fi
    echo "== $name: $sats sats, $gs gs, --rr $rr, --islrange $islrange"
    if [ ! -e "$name.scenario.json" ]; then
        "$PY" -m tvgutil.tools.create_rr_scenario --satdbfile "$TLE" --maxrot 16 \
            -s "$sats" -g "$gs" --seed "$SEED" -t "$START" -o "$name.scenario.json"
    fi
    "$PY" -m tvgutil.tools.create_rr_tvg "$name.scenario.json" \
        --rr "$rr" --islrange "$islrange" -U 9600 -o "$name.json"
done <<EOF
rr0_s40_g40 40 40 0 0
rrs_s40_g40 40 40 s 1000
rrs_s80_g20 80 20 s 1000
rrs_s80_g80 80 80 s 1000
rrs_s80_g10 80 10 s 1000
rrs_s80_g05 80 5 s 1000
EOF
