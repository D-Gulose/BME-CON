###############################################################################
# Startup code
#
# Initializes the stack pointer, calls main, and stops simulation.
#
# Memory layout:
#   0 ... ~0x300  program
#   0x7EC       Top of stack, growing down
#   0x7FC       stdin/stdout
#
###############################################################################

.org 0x00
_start:
  ADDI sp, zero, 0x7EC
  ADDI fp, sp, 0

  # set saved registers to unique default values
  # to make checking for correct preservation easier
  LUI  s1, 0x11111
  ADDI s1, s1, 0x111
  ADD  s2, s1, s1
  ADD  s3, s2, s1
  ADD  s4, s3, s1
  ADD  s5, s4, s1
  ADD  s6, s5, s1
  ADD  s7, s6, s1
  ADD  s8, s7, s1
  ADD  s9, s8, s1
  ADD  s10, s9, s1
  ADD  s11, s10, s1

  JAL  ra, main
  EBREAK

swap:
  # Prolog
  ADDI sp, sp, -32
  SW ra, 28(sp)
  
  LW t5, 0(a1)
  LW t6, 0(a2)
  SW t6, 0(a1)
  SW t5, 0(a2)

  # Epilog
  LW   ra, 28(sp)
  ADDI sp, sp, 32
  JALR zero, 0(ra)

partition:
  # Prolog
  ADDI sp, sp, -32
  SW ra, 28(sp)

  # Main
  SW s1, 24(sp)
  SW s2, 20(sp)
  SW s3, 16(sp)
  SW s4, 12(sp)
  SW s5, 8(sp)

  SW a1, 4(sp)
  SW a2, 0(sp)

  ADDI s4, zero, 2
  SLL t1, a2, s4
  ADD s5, a0, t1
  LW s2, 0(s5)

  ADDI s1, a1, 0
  ADDI s3, a1, -1

.loop_begin:
  BGE s1, a2, .loop_end
  SLL t0, s1, s4
  ADD t4, a0, t0
  LW t6, 0(t4)

  BGE t6, s2, .loop_increment
  ADDI s3, s3, 1
  SLL t1, s3, s4
  ADD a1, a0, t1
  ADDI a2, t4, 0
  JAL ra, swap
  LW a1, 4(sp)
  LW a2, 0(sp)
  JAL zero, .loop_increment

.loop_increment:
  ADDI s1, s1, 1
  JAL zero, .loop_begin

.loop_end:
  ADDI s3, s3, 1
  SLL t1, s3, s4
  ADD a1, a0, t1
  ADDI a2, s5, 0
  JAL ra, swap
  LW a1, 4(sp)
  LW a2, 0(sp)
  ADDI a0, s3, 0 # return k

  LW s1, 24(sp)
  LW s2, 20(sp)
  LW s3, 16(sp)
  LW s4, 12(sp)
  LW s5, 8(sp)

  # Epilog
  LW   ra, 28(sp)
  ADDI sp, sp, 32
  JALR zero, 0(ra)

###############################################################################
# Function: void qsort(int* A, int l, int r)
#
# Quicksort selects an element as pivot and partitions the other elements into two sub-arrays
# The sub-arrays are then sorted recursively
#
###############################################################################
qsort:
  # Prolog
  ADDI sp, sp, -32
  SW ra, 28(sp) # save return adress at sp+28;

  # Main
  SW a2, 20(sp)
  SW a0, 12(sp)
  BGE a1, a2, .end_sort

.continue_sort:
  JAL ra, partition
  ADDI a3, a0, 0
  LW a0, 12(sp)

  ADDI a2, a3, -1
  JAL ra, qsort

  ADDI a1, a3, 1
  LW a2, 20(sp)
  JAL ra, qsort
  LW a2, 20(sp)

.end_sort:
  # Epilog
  LW ra, 28(sp)
  ADDI sp, sp, 32
  JALR zero, 0(ra)

###############################################################################
# Function: int input(int *A)
#
# Reads at most 10 values from stdin to the input array.
#
# Input args:
# a0: address for array A
# Return value:
# a0: Number of read elements
#
###############################################################################
input:
  ADDI t0, a0, 0                  # Save a0
  LW   a0, 0x7fc(zero)            # Load size
  ADDI t1, zero, 10               # Maximum
  ADDI t2, zero, 0                # Loop counter
.before_input_loop:
  BGE  t2, t1, .after_input_loop  # Maximum values reached
  BGE  t2, a0, .after_input_loop  # All values read

  # Read from stdin in store in array A
  LW   t3, 0x7fc(zero)
  SW   t3, 0(t0)
  # Pointer increments
  ADDI t0, t0, 4

  ADDI t2, t2, 1                  # Increment loop counter
  JAL  zero, .before_input_loop   # Jump to loop begin

.after_input_loop:
  JALR zero, 0(ra)

###############################################################################
# Function: void output(int size, int* A)
#
# Prints input and output values to stdout
#
# Input args:
# a0: Number of elements
# a1: address for array A
#
###############################################################################
output:
.before_output_loop:
  BEQ  a0, zero, .after_output_loop
  # Load values
  LW   t0, 0(a1)
  # Output Values to stdout
  SW   t0, 0x7fc(zero)
  # Pointer increments
  ADDI a1, a1, 4
  # Decrement loop counter
  ADDI a0, a0, -1
  # jump to beginning
  JAL  zero, .before_output_loop

.after_output_loop:
  JALR zero, 0(ra)

###############################################################################
# Function: main
#
# Calls input, qsort, and output
#
###############################################################################
main:
  ADDI sp, sp, -64
  SW   ra, 60(sp)
  SW   s0, 56(sp)
  ADDI s0, sp, 64

  ADDI a0, s0, -52                # array A
  JAL  ra, input
  SW   a0, -56(s0)                # size

  ADDI a2, a0, -1                 # size - 1
  ADDI a1, zero, 0                # 0
  ADDI a0, s0, -52                # array A
  JAL  ra, qsort

  LW   a0, -56(s0)                # size
  ADDI a1, s0, -52                # array A
  JAL  ra, output

  ADDI a0, zero, 0                # return 0;

  LW   s0, 56(sp)
  LW   ra, 60(sp)
  ADDI sp, sp, 64
  JALR zero, 0(ra)
