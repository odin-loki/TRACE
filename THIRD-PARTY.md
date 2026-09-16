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

## Qt 6 (or Qt 5) — optional, `-DTRACE_WITH_QT=ON`

- **Path:** not vendored. Found with `find_package(Qt6 6.4 COMPONENTS Widgets)`,
  falling back to `Qt5 5.15`.
- **Upstream:** https://www.qt.io
- **Licence:** LGPL-3.0-or-later or GPL-3.0-or-later, at the Qt Company's
  option, or a commercial licence.
- **What links it:** `trace_console` only — the operator console under
  `src/apps/gui/`. Nothing in `trace_core`, the four command-line applications
  or the test suite touches Qt, and the option is **off** by default, so a
  default build links none of it.
- **Compatibility:** LGPL-3.0 permits conveying the combined work under
  GPL-3.0, and GPL-3.0 is one-way compatible with AGPL-3.0 through AGPL
  section 13, so `trace_console` may be distributed under AGPL-3.0. That
  direction matters: the combination is AGPL, not LGPL, and anyone
  redistributing the console owes the AGPL's obligations on the whole of it.
- **If you link Qt dynamically and distribute the console**, LGPL section 4
  additionally requires that a recipient be able to relink against a modified
  Qt. Building with `-DTRACE_WITH_QT=OFF`, which is the default, avoids the
  question entirely.

This entry was missing until the release audit. `trace_console` was linking Qt
and this file, which the README points at as the list of what is not this
project's work, did not mention it.

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

  **The citation, discharged rather than merely described.** This file used to
  say the terms "require citation" and then not give one, which is a worse
  position than not mentioning it. Every MOTChallenge number in
  [`docs/VALIDATION.md`](docs/VALIDATION.md) rests on:

  > Milan, A., Leal-Taixé, L., Reid, I., Roth, S., and Schindler, K.
  > *MOT16: A Benchmark for Multi-Object Tracking.* arXiv:1603.00831, 2016.
  > — the benchmark MOT17 extends.

  > Dendorfer, P., Rezatofighi, H., Milan, A., Shi, J., Cremers, D., Reid, I.,
  > Roth, S., Schindler, K., and Leal-Taixé, L. *MOT20: A benchmark for multi
  > object tracking in crowded scenes.* arXiv:2003.09003, 2020.

  The metrics themselves are not MOTChallenge's either:

  > Bernardin, K., and Stiefelhagen, R. *Evaluating Multiple Object Tracking
  > Performance: The CLEAR MOT Metrics.* EURASIP Journal on Image and Video
  > Processing, 2008. — MOTA, MOTP, and the match-continuity rule this
  > repository had to correct itself against.
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
