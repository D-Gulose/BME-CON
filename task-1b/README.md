# Task 1b
I completed task 1b
I did
- the divider in divider.sv
- integration of the divider into micro_riscv.sv
    * added cpu_stall logic
    * added sign extension for the quotient
    * added logic for DIVU to bypass the ALU und rather use the divider
- including the DIVU opcode in the cpu_package.sv
