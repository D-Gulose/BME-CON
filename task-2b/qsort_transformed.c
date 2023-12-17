#include <stdio.h>

//-----------------------------------------------------------------------------
// RISC-V Register set
const size_t zero = 0;
size_t a0, a1;                      // fn args or return args
size_t a2, a3, a4, a5, a6, a7;      // fn args
size_t t0, t1, t2, t3, t4, t5, t6;  // temporaries
// Callee saved registers, must be stacked befor using it in a function!
size_t s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11;
//-----------------------------------------------------------------------------

void swap(void)
{
    t5 = *(int*)a1;
    t6 = *(int*)a2;
    *(int*)a1 = t6;
    *(int*)a2 = t5;
    return;
}

void partition(void) // a1=l, a2=r, a3=k
{
    size_t stack_s1 = s1;
    size_t stack_s2 = s2; 
    size_t stack_s3 = s3;
    size_t stack_s4 = s4;
    size_t stack_s5 = s5;
    
    size_t stack_a1 = a1;
    size_t stack_a2 = a2;

    s4 = 2;         
    t1 = a2 << s4;  // shift pointer r*4
    s5 = a0 + t1;   // A+r .. &A[r]
    s2 = *(int*)s5; // A[r] (pivot)

    s1 = a1;        // j = l
    s3 = a1 - 1;    // i = l-1; 

loop_begin:
    if ((int)s1 >= (int)a2) goto loop_end;
    t0 = s1 << s4;  // j*4
    t4 = a0 + t0;   // A+j
    t6 = *(int*)t4; // A[j]

    if ((int)t6 >= (int)s2) goto loop_increment; // A[j] >= A[r] pivot
    s3 = s3 + 1;    // increment i
    t1 = s3 << s4;  // i*4
    a1 = a0 + t1;   // A+i 
    a2 = t4;        // A+j
    swap();
    a1 = stack_a1;
    a2 = stack_a2;
    goto loop_increment;

loop_increment:
    s1 = s1 + 1;
    goto loop_begin;

loop_end:
    s3 = s3 + 1;    // increment i
    t1 = s3 << s4; 
    a1 = a0 + t1;
    a2 = s5;
    swap();
    a1 = stack_a1;
    a2 = stack_a2;
    a0 = s3; // return k 

    s1 = stack_s1;
    s2 = stack_s2;
    s3 = stack_s3;
    s4 = stack_s4;
    s5 = stack_s5;
    return;
}

void qsort(void) // a0=A, a1=l, a2=r, a3=k
{
    size_t stack_a0 = a0;
    size_t stack_a2 = a2; 
    
    if ((int)a1 >= (int)a2) goto end_sort; // l < r

    partition();
    a3 = a0;
    a0 = stack_a0;
    a2 = a3-1;
    qsort();
    a1 = a3+1;
    a2 = stack_a2;
    qsort();

    a1 = stack_a0;
    a2 = stack_a2;

end_sort:
    return;
}

void input(void)
{
    // Read size
    t0 = a0; // Save a0
    a0 = fscanf(stdin, "%08x\n", (int*)&t1);
    t4 = 1;
    if (a0 == t4) goto input_continue;
    // Early exit
    a0 = 0;
    return;

input_continue:
    t4 = 1;
    t5 = 10;
input_loop_begin:
    if(t5 == 0) goto after_input_loop;
    a0 = fscanf(stdin, "%08x\n", (int*)&t2);
    if(a0 == t4) goto continue_read;
    // Exit, because read was not successful
    a0 = t1;
    return;
continue_read:
    *(int*)t0 = t2;
    // Pointer increment for next iteration
    t0 = t0 + 4;
    // Loop counter decrement
    t5 = t5 - 1;
    goto input_loop_begin;

after_input_loop:
    a0 = t1;
    return;
}

void output(void)
{
before_output_loop:
    if (a0 == 0) goto after_output_loop;

    fprintf(stdout, "%08x\n", (unsigned int)*(int*)a1);

    // Pointer increment for next iteration
    a1 = a1 + 4;
    // Decrement loop counter
    a0 = a0 - 1;
    goto before_output_loop;

after_output_loop:
    return;
}

int main(void)
{
  int A[10];
  int size;

  a0 = (size_t) A; // casting to match register a
  input();
  size = a0;

  a0 = (size_t) A;  
  a1 = 0;           // l
  a2 = size - 1;    // r
  qsort();

  a0 = size;
  a1 = (size_t) A;
  output();

  return 0;
}
