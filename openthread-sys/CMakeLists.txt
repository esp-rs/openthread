cmake_minimum_required(VERSION 3.16.0)
project(openthread VERSION 1.0.0)

# To avoid name clashes if these C functions are defined by some other library
add_definitions(-Dsnprintf=_support_snprintf -Dvsnprintf=_support_vsnprintf)

# OpenThread itself (upstream, unmodified git submodule)
add_subdirectory(./openthread)

# Custom C code (`snprintf`/`vsnprintf`, and an empty stub source)
add_subdirectory(./gen/support)

# Neutralize the OpenThread CLI libraries. OpenThread's NCP libraries
# unconditionally `target_link_libraries(... openthread-cli-*)` even though the
# CLI is only *used* when `OPENTHREAD_CONFIG_NCP_CLI_STREAM_ENABLE` is set
# (off by default here). CMake still *builds* the CLI as a link dependency, and
# `cli_tcp.cpp` fails to compile against our trimmed MbedTLS profile (it needs
# X.509). Since the CLI symbols are never referenced, we replace the CLI
# targets' sources with a single empty translation unit: the link is satisfied,
# nothing problematic is compiled, and the upstream submodule stays unpatched.
set(_empty_src "${CMAKE_CURRENT_SOURCE_DIR}/gen/support/src/empty.c")
foreach(_cli_target openthread-cli-mtd openthread-cli-ftd openthread-cli-radio)
    if(TARGET ${_cli_target})
        set_target_properties(${_cli_target} PROPERTIES SOURCES "${_empty_src}")
    endif()
endforeach()

# Umbrella target listing exactly the static libraries `openthread-sys` ships
# and links (see `gen/features.rs::device_link_libs`). The build script builds
# THIS target instead of CMake's default `ALL`, so OpenThread's CLI / RCP /
# radio example libraries — which it registers unconditionally via
# `add_subdirectory(cli)` etc. and which fail to compile against our trimmed
# MbedTLS profile (e.g. `cli_tcp.cpp` needs X.509) — are never built. We don't
# patch the upstream submodule; we just don't ask CMake to build those targets.
#
# Each library is only added if its target exists (a given OT_* configuration
# may not produce all of them).
set(_ot_sys_libs
    # Core stacks (one is linked per build, selected by the `ftd` feature).
    openthread-mtd
    openthread-ftd
    # Remote-radio (RCP host) support: the RadioSpinel client + common spinel
    # codec + HDLC framing. Linked only when the `rcp` feature is active.
    openthread-radio-spinel
    openthread-spinel-rcp
    openthread-hdlc
    # OpenThread's TCP implementation (linked only with the `tcp` feature).
    tcplp-mtd
    tcplp-ftd
    # Always linked.
    openthread-platform
    openthread-platform-utils-static
    support
)
# Note: the NCP role (`openthread-ncp-*`, `openthread-spinel-ncp`) is
# intentionally omitted — see `Cargo.toml` / `gen/features.rs`. Leaving these
# targets out of the umbrella means CMake never builds them.

set(_ot_sys_deps)
foreach(_lib ${_ot_sys_libs})
    if(TARGET ${_lib})
        list(APPEND _ot_sys_deps ${_lib})
    endif()
endforeach()

add_custom_target(openthread-sys-libs DEPENDS ${_ot_sys_deps})
