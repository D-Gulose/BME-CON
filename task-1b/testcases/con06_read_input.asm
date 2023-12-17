# micro riscv IO demo
    .org 0x00    
L0:        
    LW x1, 0x7fc(x0)       # load input                 
    BEQ x1,x0, L1          # branch to L1, if input is zero
    SW x1, 0x7fc(x0)       # write to output
    JAL x0,L0              # unconditional branch to L0
L1:
    EBREAK
