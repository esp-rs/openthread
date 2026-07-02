add_library(support src/snprintf.c)

# Minimal spinel codec shim for the async RCP-host radio driver.
#
# When `OT_RCP_HOST_SHIM` is ON (driven by the `rcp` cargo feature, see
# `gen/builder.rs`), compile `src/spinel_codec.c` into the support library. It
# re-exports OpenThread's spinel packed-uint codec (from `spinel.c`, in the
# `openthread-spinel-rcp` library) plus a handful of structural spinel constants
# as plain `extern "C"` functions that the Rust `SpinelRadio` driver
# (`openthread::rcp`) calls; the rest of the spinel/HDLC framing is done in Rust.
#
# The shim only needs to resolve `#include "lib/spinel/spinel.h"` (a pure
# enum/macro + prototype header — its optional `SPINEL_PLATFORM_HEADER` is left
# undefined here), so the sole required include root is `openthread/src`.
# Linking `openthread-spinel-rcp` + `ot-config` orders the build after them and
# carries their interface config; the codec symbols themselves are resolved at
# the final Rust link step, which pulls the `openthread-spinel-rcp` archive when
# the `rcp` feature selects it.
if(OT_RCP_HOST_SHIM)
    target_sources(support PRIVATE src/spinel_codec.c)

    target_include_directories(support PRIVATE
        ${PROJECT_SOURCE_DIR}/openthread/src
    )

    target_link_libraries(support PRIVATE openthread-spinel-rcp ot-config)
endif()
