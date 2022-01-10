// SoC specific definitions
#pragma once

#include "rv32.h"

// ===== I/O Register Definitions =====
#define IO_REG(_addr) *((volatile uint32_t*)(_addr))
#define IO_REG_OFFSET(_baseAddr, _offs) IO_REG((_baseAddr)+(_offs))

// LED controller
#define LED_STATE       IO_REG(0x4000)	// LED status register

// External interrupt controller
#define EIC_MASK        IO_REG(0x4010)  // EIC interrupt mask register
#define EIC_CIP         IO_REG(0x4014)  // EIC clear interrupt pending register
#define EIC_IRQNUM      IO_REG(0x4018)  // EIC active interrupt number register
#define EIC_IRQBIT      IO_REG(0x401C)  // EIC active interrupt bit register

// IRQ constants
#define EIC_IRQ_TIM1_BIT    0b1
#define EIC_IRQ_TIM1_NUM    0x0
#define EIC_IRQ_TIM2_BIT    0b10
#define EIC_IRQ_TIM2_NUM    0x1

// Timers
#define TIMER_BASE              0x4020          // Base address of timer address space
#define TIMER_STRIDE            0x18            // Size of time address slices, for each timer, in bytes

#define TIMER_OFFS_CONTROL      0x0
#define TIMER_OFFS_PRETH        0x4
#define TIMER_OFFS_CNTTH        0x8
#define TIMER_OFFS_CMPV1        0xC
#define TIMER_OFFS_CMPV2        0x10
#define TIMER_OFFS_CMPV3        0x14

// Timer control bits
#define TIMER_ENABLE            0b1
#define TIMER_ENABLE_IRQ        0b10
#define TIMER_ENABLE_CMP1       0b100
#define TIMER_ENABLE_CMP2       0b1000
#define TIMER_ENABLE_CMP3       0b10000

// Timer 1
#define TIMER1_BASE             TIMER_BASE
#define TIMER1_CONTROL          IO_REG_OFFSET(TIMER1_BASE, TIMER_OFFS_CONTROL)
#define TIMER1_PRETH            IO_REG_OFFSET(TIMER1_BASE, TIMER_OFFS_PRETH)
#define TIMER1_CNTTH            IO_REG_OFFSET(TIMER1_BASE, TIMER_OFFS_CNTTH)
#define TIMER1_CMPV1            IO_REG_OFFSET(TIMER1_BASE, TIMER_OFFS_CMPV1)
#define TIMER1_CMPV2            IO_REG_OFFSET(TIMER1_BASE, TIMER_OFFS_CMPV2)
#define TIMER1_CMPV3            IO_REG_OFFSET(TIMER1_BASE, TIMER_OFFS_CMPV3)

// Timer 2
#define TIMER2_BASE             (TIMER_BASE + TIMER_STRIDE)
#define TIMER2_CONTROL          IO_REG_OFFSET(TIMER2_BASE, TIMER_OFFS_CONTROL)
#define TIMER2_PRETH            IO_REG_OFFSET(TIMER2_BASE, TIMER_OFFS_PRETH)
#define TIMER2_CNTTH            IO_REG_OFFSET(TIMER2_BASE, TIMER_OFFS_CNTTH)
#define TIMER2_CMPV1            IO_REG_OFFSET(TIMER2_BASE, TIMER_OFFS_CMPV1)
#define TIMER2_CMPV2            IO_REG_OFFSET(TIMER2_BASE, TIMER_OFFS_CMPV2)
#define TIMER2_CMPV3            IO_REG_OFFSET(TIMER2_BASE, TIMER_OFFS_CMPV3)