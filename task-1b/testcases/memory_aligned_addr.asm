# illustrates memory accesses

    .org 0x00

    # Define 3 constants we want to write
    ADDI t0, x0, 0xA     # 8-bit value = 0xA
    LUI t1, 0xC
    ADDI t1, t1, 0x8BC   # 16-bit value = 0xB8BC
    LUI t2, 0xA3A8E
    ADDI t2, t2, 0xB65   # 32-bit value = 0xA3A8DB65

    # Define 3 addresses
    ADDI a0, x0, 0x600   # 32-bit aligned address
    ADDI a1, x0, 0x601   # Unaligned byte-address
    ADDI a2, x0, 0x602   # Unaligned byte-address

    SW t0, 0(a0)
    #SW t1, 0(a1)        # ERROR unaligned -> disallowed
    #SW t2, 0(a2)        # ERROR unaligned -> disallowed

    # Write result to stdout
    SW t0, 0x7fc(zero)
    SW t1, 0x7fc(zero)
    SW t2, 0x7fc(zero)

    EBREAK
