// Extra bindgen surface for the `rcp` (RCP-host / spinel) feature.
//
// The Rust `SpinelRadio` driver (`openthread::radio::spinel`) speaks the spinel
// wire protocol directly. It needs bindings for the spinel command/status/header
// constants and the variable-length "packed-uint" codec (`spinel_packed_uint_*`,
// defined in `spinel.c` -> `libopenthread-spinel-rcp.a`). Those live in
// OpenThread's *internal* header `lib/spinel/spinel.h`, which is not part of the
// public API surface in `include.h`.
//
// This header is fed to bindgen (instead of the plain `include.h`) only when the
// `rcp` cargo feature is active — see `gen/builder.rs`, which also adds the
// `openthread/src` include dir (so `lib/spinel/spinel.h` resolves) and the
// `spinel_.*` / `SPINEL_.*` allowlist entries. Non-`rcp` builds parse only
// `include.h` and so never generate the (large) spinel binding set.
//
// `spinel.h` guards its content behind an optional `SPINEL_PLATFORM_HEADER`
// (undefined here, which is fine — it then only pulls stdint/stdbool/stdarg).

#include "include.h"

#include "lib/spinel/spinel.h"
