# Third-party components

TRACE is licensed under AGPL-3.0-or-later (see [`LICENSE`](LICENSE)). The
components below are **not** TRACE's work and keep their own terms. Nothing
here is under a licence that conflicts with AGPL-3.0: BSD-3-Clause is a
permissive licence with no copyleft, so a combined work distributes under the
AGPL while the BSD notice below is preserved, as BSD-3 requires.

---

## xsimd 13.0.0

- **Path:** `third_party/xsimd/`
- **Upstream:** https://github.com/xtensor-stack/xsimd
- **Licence:** BSD 3-Clause — verbatim copy at `third_party/xsimd/LICENSE`
- **Copyright:** Johan Mabille, Sylvain Corlay, Wolf Vollprecht, Martin Renou;
  QuantStack; Serge Guelton
- **What was vendored:** headers only (`include/`). Upstream tests, docs and CI
  configuration were removed. See `third_party/xsimd/VENDORED.md`.
- **Modifications:** none. The headers are unmodified upstream 13.0.0.
- **Optional:** building with `-DTRACE_WITH_XSIMD=OFF` selects a scalar
  fallback and links no xsimd code at all.

---

## Data that is *not* in this repository

`data/` is `.gitignore`d and no annotation file is tracked in git.
[`scripts/fetch_mot.sh`](scripts/fetch_mot.sh) downloads the MOTChallenge
label archives on demand.

- **MOT17 / MOT20 annotations** — © the MOTChallenge authors, distributed from
  https://motchallenge.net under that site's own terms, which restrict use to
  non-commercial research and require citation. Those terms bind you directly
  when you run the fetch script; they are not granted by, and are not affected
  by, TRACE's licence.
- Nothing in TRACE's build or test suite requires this data. `ctest` passes on
  a fresh clone with `data/` absent; only `trace_mot` needs it.

---

## Derivation

`reference/aria_intel.py` is kept in-tree as the Python engine this C++ port
derives from; `docs/PORTING_NOTES.md` records what changed. It is the same
author's earlier work — `reference/papers/Paper2_LE_Intel_Brief.md` names Odin
Loch as its author — so it is released here under the same
AGPL-3.0-or-later terms as the rest of the repository, and carries no separate
upstream licence because it has no separate upstream. It has no per-file
copyright header of its own; the root `LICENSE` covers it.

`reference/papers/` and `docs/` are this project's own prose, not reproduced
third-party publications. Published work referenced in them is cited by title
and author, not copied.
