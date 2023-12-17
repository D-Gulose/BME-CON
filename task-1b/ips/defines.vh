`define STDIN_FILE  "stdin.txt"
`define STDOUT_FILE "stdout.txt"
`define PAROUT_FILE "parout.txt"

// amount of memory locations to be printed:
`define PRINT_MEM_LOCATIONS 128

// for clock and reset
`define HALF_PERIOD  10
`define RESET_DELAY  (`HALF_PERIOD + 1)

`define MEM_START     32'h00000000
`define MEM_SIZE      32'h000007ec

`define PAR_IN_START   (`MEM_START + `MEM_SIZE)
`define PAR_IN_SIZE    32'h8

`define PAR_OUT_START   (`PAR_IN_START + `PAR_IN_SIZE)
`define PAR_OUT_SIZE    32'h8

`define STDOUT_START   (`PAR_OUT_START + `PAR_OUT_SIZE)
`define STDOUT_SIZE    32'h4

`define STDIN_START    (`PAR_OUT_START + `PAR_OUT_SIZE)
`define STDIN_SIZE     32'h4
