add_library(support src/snprintf.c)

# RCP-host spinel bridge shim.
#
# When `OT_RCP_HOST_SHIM` is ON (driven by the `rcp` cargo feature, see
# `gen/builder.rs`), compile `src/rcp_shim.cpp` into the support library. It
# implements the `otPlatRadio*` platform callbacks by forwarding to OpenThread's
# `RadioSpinel` client and exposes the `otRcp*` C entry points that the Rust
# `OpenThread::run_rcp` pump drives. It needs the OpenThread internal spinel/HDLC
# headers (not just the public `include/` API) and links the RadioSpinel client.
if(OT_RCP_HOST_SHIM)
    target_sources(support PRIVATE src/rcp_shim.cpp)

    # The shim includes OpenThread's *internal* spinel/HDLC headers, which in
    # turn `#include "common/..."`, `"lib/..."`, etc. Mirror the include roots
    # OpenThread's own spinel library compiles with (see
    # `openthread/src/lib/spinel/CMakeLists.txt` COMMON_INCLUDES + the public
    # includes): the public API (`include`), the config/util headers
    # (`src/include`, resolves `common/code_utils.hpp`), and the source roots
    # (`src`, `src/core`).
    target_include_directories(support PRIVATE
        ${PROJECT_SOURCE_DIR}/openthread/include
        ${PROJECT_SOURCE_DIR}/openthread/src
        ${PROJECT_SOURCE_DIR}/openthread/src/core
        ${PROJECT_SOURCE_DIR}/openthread/src/include
    )

    # `openthread-radio-spinel` provides the `RadioSpinel`/`SpinelDriver` C++
    # classes the shim drives. `ot-config` is the INTERFACE target that carries
    # OpenThread's generated config header include (`etc/cmake`) and the
    # `OPENTHREAD_*` compile definitions the internal headers require — the same
    # context OpenThread's own libraries compile with. Linking both gives the
    # shim the classes to call and the config to compile the OT headers.
    target_link_libraries(support PRIVATE openthread-radio-spinel ot-config)
endif()
