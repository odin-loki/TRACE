// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// TRACE — the library's own answer to "what SIMD ABI were you built with".
//
// Deliberately its own translation unit, compiled with the library's flags, so
// that `library_tag()` reports what trace_core was built with and never what
// the caller's headers happen to say. See trace::abi in backend/simd.hpp for
// why the question is worth asking.
#include "trace/backend/simd.hpp"

namespace trace::abi {

unsigned library_tag() { return header_tag(); }

}  // namespace trace::abi
