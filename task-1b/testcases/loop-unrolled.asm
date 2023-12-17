# example with an unrolled loop
# reads ten numbers from stdin and prints the result
    .org 0x00
    ADD x1, x0, x0   # clear x1

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    LW x2, 0x7fc(x0) # load input
    ADD x1, x1, x2   # x1 += input

    SW x1, 0x7fc(x0) # output sum
    EBREAK
