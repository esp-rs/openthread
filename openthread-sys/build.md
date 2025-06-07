# How to install ricv compiler

`git clone https://github.com/riscv/riscv-gnu-toolchain`

`sudo apt-get install autoconf automake autotools-dev curl python3 python3-pip python3-toml libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev ninja-build git cmake libglib2.0-dev libslirp-dev`

`./configure --prefix=/opt/riscv --with-arch=rv32gc --with-abi=ilp32d`

`sudo make - j16`

`sudo apt install clang`

`rustup target add riscv32imac-unknown-none-elf`

`PATH="/opt/riscv/bin:$PATH" cargo build --target riscv32imac-unknown-none-elf`