
# ALU Operations
    .org 0x00
 
    ADDI x1, x0, 0x51f  # 1311
    ADDI x2, x0, 0x997  # -1641

    ADD x3, x1, x2
    SW x3, 0x7fc(x0)

    SUB x3, x1, x2
    SW x3, 0x7fc(x0)
    SUB x3, x2, x1
    SW x3, 0x7fc(x0)

    XOR x3, x1, x2
    SW x3, 0x7fc(x0)

    OR x3, x1, x2
    SW x3, 0x7fc(x0)

    AND x3, x1, x2
    SW x3, 0x7fc(x0)

    ADDI x4, x0, 4
    SRL x3, x2, x4
    SW x3, 0x7fc(x0)

    SRA x3, x2, x4
    SW x3, 0x7fc(x0)

    SLL x3, x2, x4
    SW x3, 0x7fc(x0)

    EBREAK
