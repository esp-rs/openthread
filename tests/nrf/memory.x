MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* 4K short of the full 1024K: the last flash page (0x000FF000) holds the
     OpenThread settings image - see `src/settings.rs`. Keeping it out of the
     linker's FLASH region is what guarantees the firmware can never grow
     into it. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1020K
  RAM : ORIGIN = 0x20000000, LENGTH = 256K

  /* These values correspond to the NRF52840 with Softdevices S140 7.3.0 */
  /*
     FLASH : ORIGIN = 0x00027000, LENGTH = 868K
     RAM : ORIGIN = 0x20020000, LENGTH = 128K
  */
}