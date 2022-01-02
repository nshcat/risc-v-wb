	lui t0, %hi(0x4000)
	addi t0, t0, %lo(0x4000)
	
	li t1, 0b1010
	sw t1, 0(t0)
    
.loop:	j .loop
