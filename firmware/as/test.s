				lui t0, %hi(0x4000)
				addi t0, t0, %lo(0x4000)
				lui t1, %hi(0b1010)
				addi t1, t0, %lo(0b1010)
				sw t1, 0(t0)


.toggle_leds:	lui t0, %hi(0x4000)
				addi t0, t0, %lo(0x4000)
	
				lw t1, 0(t0)
				not t1, t1
				sw t1, 0(t0)
				j .delay
    

.delay:			rdtime t0
.delay_loop: 	rdtime t1
				sub t0, t1, t0
				lui t1, %hi(10)
				addi t1, t1, %lo(10)
				bltu t0, t1, .delay_loop
				j .toggle_leds
