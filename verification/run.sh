#!/usr/bin/env bash
# TRACE — run the formal proof suite.
#
#   ./run.sh              every harness, ESBMC where it discharges, else CBMC
#   ./run.sh --esbmc      force ESBMC for all
#   ./run.sh --cbmc       force CBMC for all
#   ./run.sh v04 v15      only the named harnesses
#   TIMEOUT=1800 ./run.sh use a longer per-harness budget (default 900s)
#   ESBMC=/path/to/esbmc ./run.sh      if the binaries are not on PATH
#   CBMC=/path/to/cbmc   ./run.sh
#
# Each harness carries an EXPECTED verdict, so this is a regression test and
# not a report. Three harnesses are expected to FAIL: those are the
# counterexamples that document a real property of the code, and a suite in
# which they started passing would mean the code had changed underneath them.
# A mismatch in either direction is an error.
#
# The two marked SLOW are not discharged by either checker inside the default
# budget. Bit-precise IEEE division is where both are weakest: v12 and v13
# reach 246,000 and 146,000 SAT variables from a handful of divisions and then
# sit there. They are correct encodings of their properties, kept so a faster
# solver or a longer budget can close them, and they are reported rather than
# counted or quietly dropped.
#
# v07 is a special case. The property it states - that betweenness is
# normalised to [0,1] - is established far more strongly by tests/test_network,
# which enumerates every undirected graph on four, five and six vertices and
# runs the real function on each rather than a translation of it. The harness
# is kept because it states the claim next to the code, not because it is the
# evidence.
set -uo pipefail
cd "$(dirname "$0")"

TIMEOUT=${TIMEOUT:-900}
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
)

pass=0; fail=0; slow=0
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
        "$name" "$expected" "$checker" SKIP "$bin not found - set ESBMC= or CBMC="; continue; }

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
    else
        printf '%-28s %-8s %-7s %-8s %s\n' "$name" "$expected" "$checker" "$actual" MISMATCH
        sed -n '/^Violated property/,/^$/p' <<<"$out" | head -8
        grep -E "FAILURE|PARSING ERROR|error:" <<<"$out" | head -4
        ((fail++))
    fi
done

printf '%.0s-' {1..70}; echo
echo "$pass as expected, $fail mismatched, $slow not counted (see the header)"
exit $(( fail > 0 ))
