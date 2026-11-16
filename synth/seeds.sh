#!/usr/bin/env bash
# Place and route build/soc.json with several seeds in parallel and report the
# achieved clock frequency for each. Run synth/build.sh first.
#
#   synth/seeds.sh [FREQ_MHZ] [SEEDS...]
set -uo pipefail
cd "$(dirname "$0")/.."

# Prefer an OSS CAD Suite unpacked in the home directory over older system tools.
if [ -d "$HOME/oss-cad-suite/bin" ]; then PATH="$HOME/oss-cad-suite/bin:$PATH"; fi

FREQ=${1:-18}
shift || true
SEEDS=${*:-1 2 3 4 5 6 7 8}

mkdir -p build/seeds
for s in $SEEDS; do
    nextpnr-ice40 --up5k --package sg48 --json build/soc.json --pcf synth/icebreaker.pcf \
        --asc build/seeds/soc_$s.asc --freq "$FREQ" --seed "$s" -q \
        --log build/seeds/seed_$s.log >/dev/null 2>&1 &
done
wait

for s in $SEEDS; do
    printf "seed %-3s %s\n" "$s" "$(grep 'Max frequency' build/seeds/seed_$s.log | tail -1 | sed -E 's/.*: ([0-9.]+ MHz).*/\1/')"
done
