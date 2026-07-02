/*
 *  Minimal spinel codec shim for the async RCP-host driver.
 *
 *  The async spinel radio driver (`OpenThread::run_rcp` in the `openthread`
 *  crate) speaks the spinel wire protocol to a stock `ot-rcp` firmware, but does
 *  NOT use OpenThread's `RadioSpinel`/`SpinelDriver` C++ classes (those block
 *  synchronously in `WaitForFrame`, which would force a `block_on` in the async
 *  host). Instead the Rust driver builds/parses spinel frames itself and does
 *  real `.await` transport I/O.
 *
 *  Almost the entire spinel frame is trivially built/parsed in Rust: the header
 *  byte is a plain bitfield, and the property payloads are little-endian scalars
 *  or a `uint16`-length-prefixed data blob (`DATA_WLEN`). The ONE non-trivial
 *  piece is spinel's variable-length "packed unsigned int" encoding (used for
 *  the command id and the property key). Rather than reimplement it in Rust,
 *  this shim re-exports OpenThread's own `spinel_packed_uint_*` codec (from
 *  `spinel.c`) as plain `extern "C"` pass-throughs — they are C-linkage already,
 *  but `spinel.h` is not part of the generated bindings (it needs a build-time
 *  platform-config macro), so we surface just what the driver needs here.
 *
 *  Compiled into the `support` library only when the `rcp` feature is active
 *  (see the support `CMakeLists.txt`, `OT_RCP_HOST_SHIM`).
 */

#include <stdint.h>
#include <stddef.h>

#include "lib/spinel/spinel.h"

/* --- spinel packed-uint (variable-length) codec ------------------------- */

/* Encode `value` as a spinel packed-uint into `buf` (capacity `cap`).
 * Returns the number of bytes written, or a negative value on error. */
int ot_spinel_uint_encode(uint8_t *buf, size_t cap, uint32_t value)
{
    return (int)spinel_packed_uint_encode(buf, (spinel_size_t)cap, (unsigned int)value);
}

/* Decode a spinel packed-uint from `buf` (length `len`) into `*out`.
 * Returns the number of bytes consumed, or a negative value on error. */
int ot_spinel_uint_decode(const uint8_t *buf, size_t len, uint32_t *out)
{
    unsigned int   value = 0;
    spinel_ssize_t n     = spinel_packed_uint_decode(buf, (spinel_size_t)len, &value);

    if (n > 0 && out != NULL)
    {
        *out = (uint32_t)value;
    }

    return (int)n;
}

/* Number of bytes `value` occupies when packed. */
int ot_spinel_uint_size(uint32_t value)
{
    return (int)spinel_packed_uint_size((unsigned int)value);
}

/* --- spinel constants the Rust driver needs ----------------------------- *
 *
 * Surfaced as functions (not consts) to keep this a plain `.c` TU that the Rust
 * side links against; the values come straight from `spinel.h`. Kept minimal —
 * the driver hard-codes the rest of the (stable) property ids from the spec
 * mirrored in `rcp.rs`, but the structural constants below are the ones most
 * prone to drift, so they are sourced from the header here.
 */

uint8_t ot_spinel_header_flag(void) { return SPINEL_HEADER_FLAG; }

uint32_t ot_spinel_cmd_reset(void) { return SPINEL_CMD_RESET; }
uint32_t ot_spinel_cmd_prop_value_get(void) { return SPINEL_CMD_PROP_VALUE_GET; }
uint32_t ot_spinel_cmd_prop_value_set(void) { return SPINEL_CMD_PROP_VALUE_SET; }
uint32_t ot_spinel_cmd_prop_value_insert(void) { return SPINEL_CMD_PROP_VALUE_INSERT; }
uint32_t ot_spinel_cmd_prop_value_is(void) { return SPINEL_CMD_PROP_VALUE_IS; }

uint32_t ot_spinel_reset_stack(void) { return SPINEL_RESET_STACK; }
uint32_t ot_spinel_status_reset_begin(void) { return SPINEL_STATUS_RESET__BEGIN; }
uint32_t ot_spinel_status_reset_end(void) { return SPINEL_STATUS_RESET__END; }
uint32_t ot_spinel_status_ok(void) { return SPINEL_STATUS_OK; }
