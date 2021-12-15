#! /bin/bash

riscv64-unknown-elf-as -march=rv32i test.s
riscv64-unknown-elf-objcopy -O binary -j ".text" a.out test.bin
xxd -g 4 -u -ps -c 4 test.bin > flash.txt
rm test.bin
rm a.out
