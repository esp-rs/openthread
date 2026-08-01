add_library(support src/snprintf.c)

# The CLI output bridge (`otr_cli_init`); only for the `cli` cargo feature,
# which is also when the real CLI libraries are built (see the top-level
# `OT_RS_CLI` block).
if(OT_RS_CLI)
    target_sources(support PRIVATE src/cli_shim.c)
    target_include_directories(support PRIVATE ${PROJECT_SOURCE_DIR}/openthread/include)
endif()
