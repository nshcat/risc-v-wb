	lui t0, %hi(0x3004)
    addi t0, t0, %lo(0x3004)
    lui t1, %hi(0xAABBCCDD)
    addi t1, t1, %lo(0xAABBCCDD)
    sw t1, 0(t0)
    lw t2, 0(t0)
	
	lbu t2, 0(t0)
	lbu t2, 1(t0)
	lbu t2, 2(t0)
	lbu t2, 3(t0)
	
	lhu t2, 0(t0)
	lhu t2, 2(t0)
    
.loop:	j .loop
