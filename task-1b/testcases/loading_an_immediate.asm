# example loading a constant
# this example uses ADDI to load in immediate value
    .org 0x00
    ADDI x1, x0, 0x42  # load immediate
    LW x2, 0x7fc(x0)   # load stdin
    
    ADD x3, x1, x2     # add them up

    SW x3, 0x7fc(x0)   # output sum
    EBREAK
