#!/bin/sh
# Stress matrix for the beacon network: every line must end in PASS.
run() {
  out=$(./bpsim --quiet "$@" 2>&1 >/dev/null)
  res=$(echo "$out" | grep -q "ALL CHECKS PASSED" && echo PASS || echo FAIL)
  lat=$(echo "$out" | grep "latency drop" | sed 's/.*gateway: //')
  duty=$(echo "$out" | grep "rolling-hour" | sed 's/.*cycle \([0-9.]*\) %.*/\1%/')
  upd=$(echo "$out" | grep -o "victim update (seq 2) reached the gateway in [0-9.]* s" | grep -o "[0-9.]* s$")
  printf "%-4s %-42s %-44s duty %-6s upd %s\n" "$res" "$*" "$lat" "$duty" "${upd:-n/a}"
}
for s in 1 2 3 4 5 6 7 8 9 10; do run --seed $s; done
for s in 1 2 3; do run --beacons 30 --minutes 90 --seed $s; done
run --beacons 38 --minutes 120
for sf in 7 8 10 11 12; do run --sf $sf; done
for s in 1 2 3 4 5; do run --rubble 2.5 --seed $s; done
for s in 1 2 3 4 5 6; do run --rubble 2.5 --beacons 30 --minutes 90 --seed $s; done
run --round 30
run --round 180
run --no-attacker
run --digest 0
run --no-seed
run --no-seed --rubble 2.5 --beacons 30 --minutes 90 --seed 4
run --kill 1@20
run --kill 12@15
run --gw-late 40
run --gw-late 40 --beacons 30 --minutes 70
run --sf 12 --minutes 120
run --sf 12 --minutes 180 --kill 3@50
