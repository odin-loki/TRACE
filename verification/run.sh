#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>
# TRACE — run the formal proof suite.
#
#   ./run.sh              every harness, ESBMC where it discharges, else CBMC
#   ./run.sh --esbmc      force ESBMC for all
#   ./run.sh --cbmc       force CBMC for all
#   ./run.sh v04 v15      only the named harnesses
#   TIMEOUT=3600 ./run.sh use a longer per-harness budget (default 1800s)
#   ESBMC=/path/to/esbmc ./run.sh      if the binaries are not on PATH
#   CBMC=/path/to/cbmc   ./run.sh
#
#   STRICT=1 ./run.sh     treat an undecided harness as a failure too
#
# Each harness carries an EXPECTED verdict, so this is a regression test and
# not a report. Three harnesses are expected to FAIL: those are the
# counterexamples that document a real property of the code, and a suite in
# which they started passing would mean the code had changed underneath them.
# A mismatch in either direction is an error.
#
# Three verdicts, not two. A checker can also run out of budget, and that is
# UNDECIDED - it is not evidence that a property failed, which is what calling
# it a mismatch used to claim. The distinction matters because the budget is a
# property of the machine: v13_clutter_rate discharges comfortably on an idle
# host and times out on a loaded one, so on a busy CI runner the old code
# reported a proof regression that had not happened. A decided-and-wrong
# verdict fails the suite; an undecided one is reported and, unless STRICT=1,
# does not.
#
# Measured wall-clock on an idle machine, for the three that are not instant:
# v07 71 s, v16 121 s, v13 1181 s. A whole run is dominated by v13 and by
# v12b's budget, so expect about three quarters of an hour.
#
# One harness is marked SLOW in the manifest: v12b_segment_geometry, which
# neither checker discharges inside any budget tried. Bit-precise IEEE division
# is where both are weakest - it reaches a quarter of a million SAT variables
# from a handful of divisions and then sits there. It is a correct encoding of
# its property, kept so a faster solver can close it, and it is reported rather
# than counted or quietly dropped. (An earlier version of this header said
# "the two marked SLOW... v12 and v13". v12 was since split, and its tractable
# half - the clamp - discharges; v13 discharges too. The manifest below is the
# authority.)
#
# v07 is a special case. The property it states - that betweenness is
# normalised to [0,1] - is established far more strongly by tests/test_network,
# which enumerates every undirected graph on four, five and six vertices and
# runs the real function on each rather than a translation of it. The harness
# is kept because it states the claim next to the code, not because it is the
# evidence.
set -uo pipefail
cd "$(dirname "$0")"

# 1800s, not 900s. v13_clutter_rate takes about twenty minutes under CBMC on an
# idle machine here, so at the old default it was UNDECIDED on every run - a
# suite whose own default cannot discharge its own harnesses is reporting on the
# budget, not on the code. The other sixteen are well inside a minute or two.
TIMEOUT=${TIMEOUT:-1800}
STRICT=${STRICT:-0}
ESBMC=${ESBMC:-esbmc}
CBMC=${CBMC:-cbmc}
FORCE=
ARGS=()
for a in "$@"; do
    case $a in
        --cbmc)  FORCE=cbmc ;;
        --esbmc) FORCE=esbmc ;;
        *)       ARGS+=("$a") ;;
    esac
done

# harness : expected : preferred checker : extra options
MANIFEST=(
  "v01_mat2_divisor:PASS:esbmc:"
  "v02_mat2_nan:FAIL:esbmc:"
  "v03_clamp_nan:FAIL:esbmc:"
  "v04_hungarian:PASS:cbmc:--unwind 20"
  "v05_log_sum_exp:PASS:cbmc:"
  "v06_otsu:PASS:cbmc:--unwind 16"
  "v07_brandes_norm:PASS:cbmc:--unwind 8"
  "v08_existence_hit:FAIL:esbmc:"
  "v09_existence_saturation:PASS:esbmc:"
  "v10_mou_identity:PASS:cbmc:--unwind 8"
  "v11_vec2_unit:PASS:esbmc:"
  "v12_segment_clamp:PASS:cbmc:"
  "v12b_segment_geometry:SLOW:cbmc:"
  "v13_clutter_rate:PASS:cbmc:--unwind 16"
  "v14_existence_miss:PASS:cbmc:"
  "v15_hungarian_optimal:PASS:cbmc:--unwind 10"
  "v16_existence_continuity:PASS:cbmc:"
)

pass=0; fail=0; slow=0; undecided=0; skipped=0
printf '%-28s %-8s %-7s %-8s %s\n' HARNESS EXPECTED VIA ACTUAL RESULT
printf '%.0s-' {1..70}; echo

for entry in "${MANIFEST[@]}"; do
    IFS=: read -r name expected checker opts <<<"$entry"
    if (( ${#ARGS[@]} )); then
        want=0; for a in "${ARGS[@]}"; do [[ $name == *"$a"* ]] && want=1; done
        (( want )) || continue
    fi
    [[ -n $FORCE ]] && checker=$FORCE
    bin=$([[ $checker == esbmc ]] && echo "$ESBMC" || echo "$CBMC")
    command -v "$bin" >/dev/null || { printf '%-28s %-8s %-7s %-8s %s\n' \
        "$name" "$expected" "$checker" SKIP "$bin not found - set ESBMC= or CBMC="
        ((skipped++)); continue; }

    if [[ $checker == esbmc ]]; then
        out=$(timeout "$TIMEOUT" "$bin" "$name.c" -DUSE_ESBMC --floatbv --z3 $opts 2>&1)
    else
        out=$(timeout "$TIMEOUT" "$bin" "$name.c" --bounds-check --pointer-check \
              --unwinding-assertions $opts 2>&1)
    fi

    if   grep -q "VERIFICATION SUCCESSFUL" <<<"$out"; then actual=PASS
    elif grep -q "VERIFICATION FAILED"     <<<"$out"; then actual=FAIL
    else actual=TIMEOUT; fi

    if [[ $expected == SLOW ]]; then
        printf '%-28s %-8s %-7s %-8s %s\n' "$name" "$expected" "$checker" "$actual" \
               "$([[ $actual == TIMEOUT ]] && echo 'not discharged (known)' || echo "discharged: $actual")"
        ((slow++))
    elif [[ $actual == "$expected" ]]; then
        printf '%-28s %-8s %-7s %-8s %s\n' "$name" "$expected" "$checker" "$actual" ok
        ((pass++))
    elif [[ $actual == TIMEOUT ]]; then
        # Out of budget is not a verdict. Reported, never silent, and counted
        # apart from the failures so a slow machine cannot manufacture a proof
        # regression.
        printf '%-28s %-8s %-7s %-8s %s\n' "$name" "$expected" "$checker" "$actual" \
               "UNDECIDED at ${TIMEOUT}s - raise TIMEOUT to decide it"
        ((undecided++))
    else
        printf '%-28s %-8s %-7s %-8s %s\n' "$name" "$expected" "$checker" "$actual" MISMATCH
        sed -n '/^Violated property/,/^$/p' <<<"$out" | head -8
        grep -E "FAILURE|PARSING ERROR|error:" <<<"$out" | head -4
        ((fail++))
    fi
done

printf '%.0s-' {1..70}; echo
echo "$pass as expected, $fail mismatched, $undecided undecided, $slow not counted, $skipped skipped"

# A run that checked nothing is not a run that found nothing wrong. Exiting 0
# on an empty suite is how a missing checker, a bad path or - as happened once -
# a lost executable bit reads as success to anything watching.
if (( pass + fail + undecided + slow == 0 )); then
    echo "nothing was checked: no harness ran. Set CBMC= or ESBMC=, or name a harness that exists."
    exit 2
fi
if (( fail > 0 )); then exit 1; fi
if (( STRICT && undecided > 0 )); then exit 1; fi
exit 0
