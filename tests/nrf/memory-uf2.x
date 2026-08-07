/* Flash layout for a board with the Adafruit nRF52 UF2 bootloader
   (XIAO nRF52840, nRF52840 dongle, Feather nRF52840, ...).

   The bootloader owns the start of flash - MBR at 0, then the S140 SoftDevice
   - and jumps to the application above it. We never use the SoftDevice; we
   only have to start past it and leave it intact, which costs 156K of flash.

   RAM is left whole: the SoftDevice reserves RAM only once an application
   enables it, and this one never does. */
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x00027000, LENGTH = 868K
  RAM : ORIGIN = 0x20000000, LENGTH = 256K
}
