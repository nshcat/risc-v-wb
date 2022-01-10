.set CSR_MTVEC, 0x305
.set CSR_MIE, 0x304
.set CSR_MSTATUS, 0x300
.set EIC_MASK, 0x4010
.set EIC_CIP, 0x4014
.set EIC_IRQNUM, 0x4018
.set EIC_IRQBIT, 0x401C
.set TIM1_PRETH, 0x4024
.set TIM1_CONTROL, 0x4020
.set TIM1_CNTTH, 0x4028

		# Set ISR address
		lui t0, %hi(0x54)
		addi t0, t0, %lo(0x54)
		csrw CSR_MTVEC, t0

		# Enable external interrupt source in MIE CSR (bit 11)
		li t0, 0x800
		csrw CSR_MIE, t0

		# Enable interrupts in MSTATUS CSR (bit 3)
		li t0, 0x8
		csrw CSR_MSTATUS, t0

		# Set timer 1 counter threshold to 50
		li t0, TIM1_CNTTH
		li t1, 49
		sw t1, 0(t0)
		
		# Enable timer 1 and its interrupt
		li t0, TIM1_CONTROL
		li t1, 0b11
		sw t1, 0(t0)

		# Enable timer 1 IRQ in EIC
		li t0, EIC_MASK
		li t1, 0b1
		sw t1, 0(t0)

.loop: 	j .loop

.isr: 	# Clear bit
		li t0, EIC_IRQBIT
		lw t1, 0(t0)
		li t0, EIC_CIP
		sw t1, 0(t0)
		mret