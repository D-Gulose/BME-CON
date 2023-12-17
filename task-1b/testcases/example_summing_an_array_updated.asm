main:
      ADDI t0, zero, 0         
      LW t1, n
      LW t2, sum
      ADDI t3, zero, array    

      ADDI t4, t3, 0

      JAL zero, test          

body: LW t5, 0(t4)            
      ADD t2, t2, t5     

      ADDI t4, t4, 4     
      ADDI t0, t0, 1          
   
test: BLT t0, t1, body       

      SW t2, sum 
      SW t2, 0x7FC(zero)        
      EBREAK                 

n:     .word 4                # number of elements in n
array: .word 3, 4, 5, 6       # array to sum up
sum:   .word 0                # sum should finally be 18 == 0x12
