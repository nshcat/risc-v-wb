				li t0, 0x4000
				li t1, 0b0101
				sw t1, 0(t0)
				
.loop:			li a0, 1000
				jal .delay
				
				li t0, 0x4000
				lw t1, 0(t0)
				not t1, t1
				sw t1, 0(t0)
				j .loop
				

# Delays execution by the numnber of milliseconds passed in a0
.delay:			rdtime t0
.delay_loop: 	rdtime t1
				sub t2, t1, t0
				bltu t2, a0, .delay_loop
				ret
