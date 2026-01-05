## Folder structure:
```
pre-generated/
  {config_hash}/
    {target}/
      config.txt
      bindings.rs
      libs/
        *.a
```

## config_hash
The config hash is derived from the configuration values passed into the build of OpenThread, which in turn is derived from feature flags in this openthread-sys crate.

## target
This is the target of the build, ie. `riscv32-imac-unknown-none-elf`

## config.txt
Contains the flags that were passed into CMake during the pre-generation
