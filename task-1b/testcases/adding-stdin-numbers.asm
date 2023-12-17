# adding-stdin-numbers example
# reads two numbers from stdin and prints the sum

# example with I/O
    .org 0x00
    LW x1, 0x7fc(x0)
    LW x2, 0x7fc(x0)
    ADD x3, x1, x2
    SW x3, 0x7fc(x0)
    EBREAK
