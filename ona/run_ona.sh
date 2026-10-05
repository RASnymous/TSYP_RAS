#!/usr/bin/env bash
# Living Map - Outside Network Area (Linux / the VM). Extra options go through:
#   ./run_ona.sh --command-post http://10.0.2.2:3000        (dashboard on Windows)
cd "$(dirname "$0")" && exec python3 -m ona --udp 0.0.0.0:47100 "$@"
