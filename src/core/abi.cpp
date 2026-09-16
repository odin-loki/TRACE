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

unsigned library_tag() {
    // Bound to a constant expression on purpose, rather than `return
    // header_tag();`. The value has to be decided while THIS file is being
    // compiled - that is the only reason this file exists - and a call, even
    // to a constexpr function, is a call the linker is free to resolve
    // elsewhere. Built portable and linked from an `-mavx2` consumer at -O0,
    // the earlier `return header_tag();` returned the consumer's tag and
    // `compatible()` said yes to a genuine mismatch.
    constexpr unsigned kTag = header_tag<detail::kThisUnitsTag>();
    return kTag;
}

}  // namespace trace::abi
