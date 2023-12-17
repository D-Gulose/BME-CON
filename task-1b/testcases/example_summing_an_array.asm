# char* ref;
# int i, offset, value, old_sum, new_sum, n_value, const;

main:
      ADDI a2, x0, 2          # const = 2;
      ADDI t0, x0, 0          # i = 0;
      JAL zero, test          # goto test;

body: ADD t2, x0, t0          # offset = i;
      SLL t2, t2, a2          # offset = offset << 2;
      ADDI t1, x0, array      # ref = (char*)array_addr;
      ADD t1, t1, t2          # ref = (int*)(ref + offset);
      LW t3, 0(t1)            # value = *ref;
      LW t4, sum              # old_sum = *sum_addr;
      ADD t5, t4, t3          # new_sum = old_sum + value;
      SW t5, sum              # *sum_addr = new_sum;
      ADDI t0, t0, 1          # i++;

test: LW t6, n                # n_value = (int)*n_addr;
      BLT t0, t6, body        # if (i<n_value) goto body;

      LW t5, sum              # new_sum = *sum;
      SW t5, 0x7FC(x0)        # printf("%d\n", sum);
      EBREAK                  # return;

n:     .word 4                #// number of elements in n
array: .word 3, 4, 5, 6       #// array to sum up
sum:   .word 0                #// sum should finally be 18 == 0x12
