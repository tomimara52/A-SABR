# LEO Ring Road scenarios

Contact plans of LEO Ring Road networks (cubesats + ground stations), used by
`benches/pathfinding_benchmark.rs`. They are tvgutil P-TVG files (`PredictedContact_v2`),
parsed with `TVGUtilContactPlan::parse`.

Generated with [dtn-tvg-util](https://gitlab.com/d3tn/dtn-tvg-util) 0.1.6 by `generate.sh`:

1. `create_rr_scenario` picks the satellites from `cubesat_tle.txt` and places the ground
   stations (`gs00`, `gs01`, ...) at random positions (seed 1).
2. `create_rr_tvg` predicts the contacts with SGP4 over 24 h from 2026-10-01 13:06:40 UTC
   (`1790860000`), with a minimum elevation of 10°.

Common settings: `--maxrot 16`, uplink 9600 bit/s, downlink 250 kbit/s, ISLs at
250 kbit/s, up to 600 s each. tvgutil does not model delays: every contact delay is 0.

| Scenario | Sats | GS | ISLs | Contacts (of which ISL) | Median contact |
|---|---|---|---|---|---|
| `rr0_s40_g40` | 40 | 40 | none | 14 812 (0) | 420 s |
| `rrs_s40_g40` | 40 | 40 | ≤ 1000 km | 18 572 (3 760) | 393 s |
| `rrs_s80_g20` | 80 | 20 | ≤ 1000 km | 32 182 (15 856) | 281 s |
| `rrs_s80_g80` | 80 | 80 | ≤ 1000 km | 81 570 (15 856) | 394 s |

Contacts are unidirectional (each link appears in both directions). `<name>.scenario.json`
is the intermediate scenario (satellite TLEs, ground station positions, time offset).

## Regenerating

```sh
python3 -m venv tvgenv
tvgenv/bin/pip install "dtn-tvg-util[ring_road]==0.1.6"
./generate.sh tvgenv/bin/python
```

The output only depends on `cubesat_tle.txt`, the seed and the start time. To use newer
orbits, replace `cubesat_tle.txt` (CelesTrak updates TLEs continuously) and update `START`
in `generate.sh` to a date close to the TLE epochs:

```sh
curl -o cubesat_tle.txt "https://celestrak.org/NORAD/elements/gp.php?GROUP=cubesat&FORMAT=tle"
```

`cubesat_tle.txt` was downloaded on 2026-10-01; its TLE epochs are from 2026-09-30.

Note: the ground stations are uniformly random on the globe, so some are in the ocean.
`create_gs_list` (extra `gs_placement`, needs GEOS and GDAL) places them on land; pass its
output with `create_rr_scenario --gsfile`.
