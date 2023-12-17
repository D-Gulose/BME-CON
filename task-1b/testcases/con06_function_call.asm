# micro riscv IO demo with "subroutine"
    .org 0x00

L0:
    JAL x1, READ_WORD      # Call READ_WORD (jump to READ_WORD and store PC+4 in x1)

    BEQ x2,x0, L1          # branch to L1, if input is zero
    SW x2, 0x7fc(x0)       # write to output
    JAL x0,L0              # unconditional branch to L0
L1:
    EBREAK

READ_WORD:
    LW x2, 0x7fc(x0)       # load input
    JALR x0,0(x1)          # return to caller (return address is stored in x1)
