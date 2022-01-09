#########################################################################################################
## boot.s - Microcontroller boot and startup code
##
## Defined symbols:
##  _reset:         Reset vector. Is jumped to when µC resets. Jumps to _start.
##  _isr_common:    Common entry-point for interrupt handling. Will execute user-specified
##                  ISR according to interrupt vector table.
##  _irq_reg_save:  Scratch space for saved register contents during ISR execution
##  _start:         Main entry point. Sets up stack, .data and .bss contents, and calls C main().
##
## The C side of things is required to define at least two symbols:
##  main():         The C main function.
##  isr_vector:     Interrupt vector table: Read-only array of function pointers to user-specified
##                  ISRs.
##  exc_handler:    Function handling hardware fault exception. Takes the cause and exception detail value
##                  as uint32_t parameters.
#########################################################################################################

# Constants

# CSRS
.set CSR_MTVEC, 0x305
.set CSR_MSCRATCH, 0x340
.set CSR_MEPC, 0x341
.set CSR_MCAUSE, 0x342
.set CSR_MTVAL, 0x343

# Peripheral memory-mapped registers


# Reset vector. Must be placed at address 0x0.
.align 4
.section .reset,"ax",%progbits   # The "ax" is required to make this section actually occupy memory when converting
.global _reset                   # to a flat binary
_reset:
    j _start

        
# Safe space for register contents during ISR execution
.align 4
.section .bss
.global _irq_reg_save
_irq_reg_save: .fill 20*4

# Common IRQ handler. Must be placed at address 0x10.
.align 4
.section .isr_common,"ax",%progbits
.global _isr_common
_isr_common:
    # Save all registers
    # We use MSCRATCH to save t0
    csrw CSR_MSCRATCH, t0

    # Now we can use t0 to build the address for the register contents save location
    lui t0, %hi(_irq_reg_save)
    addi t0, t0, %lo(_irq_reg_save)
    
    # Store all registers, other than t0
    sw x1, 0*4(t0)
    sw x2, 1*4(t0)
    sw x3, 2*4(t0)
    sw x4, 3*4(t0)
    sw x6, 4*4(t0) # x5 is t0
    sw x7, 5*4(t0)
    sw x8, 6*4(t0)
    sw x9, 7*4(t0)
    sw x10, 8*4(t0)
    sw x11, 9*4(t0)
    sw x12, 10*4(t0)
    sw x13, 11*4(t0)
    sw x14, 12*4(t0)
    sw x15, 13*4(t0)
    sw x16, 14*4(t0)
    sw x17, 15*4(t0) # We don't have to save the saved registers, since that's the job of the ISR function
    sw x28, 16*4(t0)
    sw x29, 17*4(t0)
    sw x30, 18*4(t0)
    sw x31, 19*4(t0)
    
    # Check if this was an interrupt. The uppermost bit is set in that case.
    csrr t0, CSR_MCAUSE
    bltz t0, .was_interrupt

    # For exceptions, call exception handler
    mv a0, t0           # First argument is MCAUSE
    csrr a1, CSR_MTVAL  # Second argument are exception details
    call exc_handler
    j .restore

.was_interrupt:
    
    
.restore:
    # Restore all registers
    # Build base address. The called ISR could have clobbered t0, so we have to recalculate it.
    lui t0, %hi(_irq_reg_save)
    addi t0, t0, %lo(_irq_reg_save)
    
    # Load most registers, except t0 and the saved registers
    lw x1, 0*4(t0)
    lw x2, 1*4(t0)
    lw x3, 2*4(t0)
    lw x4, 3*4(t0)
    lw x6, 4*4(t0)
    lw x7, 5*4(t0)
    lw x8, 6*4(t0)
    lw x9, 7*4(t0)
    lw x10, 8*4(t0)
    lw x11, 9*4(t0)
    lw x12, 10*4(t0)
    lw x13, 11*4(t0)
    lw x14, 12*4(t0)
    lw x15, 13*4(t0)
    
#ifndef RV32E
    lw x16, 14*4(t0)
    lw x17, 15*4(t0)
    lw x28, 16*4(t0)
    lw x29, 17*4(t0)
    lw x30, 18*4(t0)
    lw x31, 19*4(t0)
#endif

    # Restore t0 from MSCRATCH
    csrr t0, CSR_MSCRATCH

    # Return from ISR.
    mret

.align 4
.global _start
.section .text
_start:
	# Setup stack pointer
	li sp, 0x3FFF

    # Setup interrupt and exception handler. The linker script places it at 0x10.
    li t0, 0x10
    csrw CSR_MTVEC, t0
	
	# Copy .data segment initializers
	lui t0, %hi(_sidata)		# Data source begin
	addi t0, t0, %lo(_sidata)		
	
	lui t1, %hi(_sdata)			# Data destination begin
	addi t1, t1, %lo(_sdata)
	
	lui t2, %hi(_edata)			# Data destination end
	addi t2, t2, %lo(_edata)
	
.copy_loop:
	bgeu t1, t2, .copy_end		# Stop if cur_dest > dest_end
	lw s1, 0(t0)                # We use s1 here instead of t3 to support RV32E
	sw s1, 0(t1)
	
	addi t0, t0, 4
	addi t1, t1, 4
	
	j .copy_loop
	
.copy_end:

	# Clear .bss section. It has to be filled with zeroes.
	lui t0, %hi(_sbss)			# i = BSS begin
	addi t0, t0, %lo(_sbss)		
	
	lui t1, %hi(_ebss)			# BSS end
	addi t1, t1, %lo(_ebss)
	
.zero_loop:
	bgeu t0, t1, .zero_end		# Stop if i >= BSS end
	sw zero, 0(t0)				# memory[i] = 0
	addi t0, t0, 4				# i = i + 4
	j .zero_loop				# continue with next loop iteration

.zero_end:
	# Jump to main function and actually execute the program
	jal main
	
.loop:
	j .loop  # Main function should never return; If it does, loop here forever
