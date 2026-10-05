#!/usr/bin/env bash
# Zone simulator -> ONA over UDP. Faults: --liar gw3 --forge gw2@60 --kill 6@90 --slam-slip 150:2.5,-1.5
cd "$(dirname "$0")" && exec python3 -m ona.zonesim --speed 2 "$@"
