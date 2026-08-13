/**
 * Bridge between OpenThread's CLI output callback and Rust.
 *
 * The CLI reports its output through a printf-style callback taking a
 * `va_list` (`otCliOutputCallback`), which Rust cannot receive. This shim
 * formats each callback invocation into a stack buffer and forwards the
 * resulting bytes to a Rust function with a C-friendly signature.
 *
 * Compiled into `libsupport.a` only for the `cli` cargo feature
 * (`OT_RS_CLI=ON`); `otr_cli_output` is then provided by the `openthread`
 * crate (its `cli` module).
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#include <openthread/cli.h>

/* Implemented in Rust (`openthread::cli`). */
extern void otr_cli_output(void *aContext, const char *aOutput, size_t aLength);

/*
 * Sized for the largest single `OutputFormat` invocation the CLI makes: one
 * output line, whose worst case is a full Operational Dataset as one hex
 * string (254 TLV bytes = 508 characters).
 */
#define CLI_OUTPUT_BUF_SIZE 1024

static int cli_output_callback(void *aContext, const char *aFormat, va_list aArguments)
{
    char buf[CLI_OUTPUT_BUF_SIZE];
    int  len = vsnprintf(buf, sizeof(buf), aFormat, aArguments);

    if (len > 0)
    {
        size_t out_len = (size_t)len;

        if (out_len > sizeof(buf) - 1)
        {
            out_len = sizeof(buf) - 1;
        }

        otr_cli_output(aContext, buf, out_len);
    }

    return len;
}

void otr_cli_init(otInstance *aInstance, void *aContext)
{
    otCliInit(aInstance, cli_output_callback, aContext);
}
