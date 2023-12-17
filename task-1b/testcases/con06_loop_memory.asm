# micro riscv loop demo with symbols
    .org 0x00
    JAL x0, main

    .org 0x120
counter: .word 0
sum:     .word 0

    .org 0x3a0
main:
    ADDI x4, x0, 10        # iteration count
                           #    +-----------------------+
loop_start:                #    v                       |
    LW x1, sum             # load sum                   |
    LW x2, 0x7fc(x0)       # load input                 |
    ADD x1, x1, x2         # sum += input               |
    SW x1, sum             # store sum                  |
                           #                            |
    LW x3, counter         # load counter               |
    ADDI x3, x3, 1         # counter++                  |
    SW x3, counter         # store counter              |
    BLT x3, x4, loop_start # if (counter < 10) goto loop_start

    SW x1, 0x7fc(x0)       # output sum
    EBREAK
