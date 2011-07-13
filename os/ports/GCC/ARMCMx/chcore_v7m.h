/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    GCC/ARMCMx/chcore_v7m.h
 * @brief   ARMv7-M architecture port macros and structures.
 *
 * @addtogroup ARMCMx_V7M_CORE
 * @{
 */

#ifndef _CHCORE_V7M_H_
#define _CHCORE_V7M_H_

/*===========================================================================*/
/* Port constants.                                                           */
/*===========================================================================*/

/**
 * @brief   Disabled value for BASEPRI register.
 */
#define CORTEX_BASEPRI_DISABLED         0

/*===========================================================================*/
/* Port configurable parameters.                                             */
/*===========================================================================*/

/**
 * @brief   Simplified priority handling flag.
 * @details Activating this option will make the Kernel work in compact mode.
 */
#ifndef CORTEX_SIMPLIFIED_PRIORITY
#define CORTEX_SIMPLIFIED_PRIORITY      FALSE
#endif

/**
 * @brief   SVCALL handler priority.
 * @note    The default SVCALL handler priority is defaulted to
 *          @p CORTEX_MAXIMUM_PRIORITY+1, this reserves the
 *          @p CORTEX_MAXIMUM_PRIORITY priority level as fast interrupts
 *          priority level.
 */
#ifndef CORTEX_PRIORITY_SVCALL
#define CORTEX_PRIORITY_SVCALL          (CORTEX_MAXIMUM_PRIORITY + 1)
#else
/* If it is externally redefined then better perform a validity check on it.*/
#if !CORTEX_IS_VALID_PRIORITY(CORTEX_PRIORITY_SVCALL)
#error "invalid priority level specified for CORTEX_PRIORITY_SVCALL"
#endif
#endif

/*===========================================================================*/
/* Port derived parameters.                                                  */
/*===========================================================================*/

/**
 * @brief   BASEPRI level within kernel lock.
 * @note    In compact kernel mode this constant value is enforced to zero.
 */
#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
#define CORTEX_BASEPRI_KERNEL                                               \
  CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_SVCALL+1)
#else
#define CORTEX_BASEPRI_KERNEL           0
#endif

/**
 * @brief   PendSV priority level.
 * @note    This priority is enforced to be equal to @p CORTEX_BASEPRI_KERNEL,
 *          this handler always have the highest priority that cannot preempt
 *          the kernel.
 */
#define CORTEX_PRIORITY_PENDSV          CORTEX_BASEPRI_KERNEL

/*===========================================================================*/
/* Port exported info.                                                       */
/*===========================================================================*/

#if (CORTEX_MODEL == CORTEX_M3) || defined(__DOXYGEN__)
/**
 * @brief   Macro defining the specific ARM architecture.
 */
#define CH_ARCHITECTURE_ARM_v7M

/**
 * @brief   Name of the implemented architecture.
 */
#define CH_ARCHITECTURE_NAME            "ARMv7-M"

/**
 * @brief   Name of the architecture variant.
 */
#define CH_CORE_VARIANT_NAME            "Cortex-M3"

#elif (CORTEX_MODEL == CORTEX_M4)
#define CH_ARCHITECTURE_ARM_v7ME
#define CH_ARCHITECTURE_NAME            "ARMv7-ME"
#define CH_CORE_VARIANT_NAME            "Cortex-M4"
#endif

/**
 * @brief   Port-specific information string.
 */
#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
#define CH_PORT_INFO                    "Advanced kernel mode"
#else
#define CH_PORT_INFO                    "Compact kernel mode"
#endif

/*===========================================================================*/
/* Port implementation part.                                                 */
/*===========================================================================*/

#if !defined(_FROM_ASM_)

/**
 * @brief   Generic ARM register.
 */
typedef void *regarm_t;

#if !defined(__DOXYGEN__)
struct extctx {
  regarm_t      r0;
  regarm_t      r1;
  regarm_t      r2;
  regarm_t      r3;
  regarm_t      r12;
  regarm_t      lr_thd;
  regarm_t      pc;
  regarm_t      xpsr;
};

struct intctx {
  regarm_t      r4;
  regarm_t      r5;
  regarm_t      r6;
  regarm_t      r7;
  regarm_t      r8;
  regarm_t      r9;
  regarm_t      r10;
  regarm_t      r11;
  regarm_t      lr;
};
#endif

/**
 * @brief   IRQ prologue code.
 * @details This macro must be inserted at the start of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#define PORT_IRQ_PROLOGUE()

/**
 * @brief   IRQ epilogue code.
 * @details This macro must be inserted at the end of all IRQ handlers
 *          enabled to invoke system APIs.
 */
#define PORT_IRQ_EPILOGUE() _port_irq_epilogue()

/**
 * @brief   IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#define PORT_IRQ_HANDLER(id) void id(void)

/**
 * @brief   Fast IRQ handler function declaration.
 * @note    @p id can be a function name or a vector number depending on the
 *          port implementation.
 */
#define PORT_FAST_IRQ_HANDLER(id) void id(void)

/**
 * @brief   Port-related initialization code.
 */
#define port_init() {                                                       \
  SCB_AIRCR = AIRCR_VECTKEY | AIRCR_PRIGROUP(0);                            \
  NVICSetSystemHandlerPriority(HANDLER_SVCALL,                              \
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_SVCALL));                          \
  NVICSetSystemHandlerPriority(HANDLER_PENDSV,                              \
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_PENDSV));                          \
  NVICSetSystemHandlerPriority(HANDLER_SYSTICK,                             \
    CORTEX_PRIORITY_MASK(CORTEX_PRIORITY_SYSTICK));                         \
}

/**
 * @brief   Kernel-lock action.
 * @details Usually this function just disables interrupts but may perform
 *          more actions.
 * @note    In this port this it raises the base priority to kernel level.
 */
#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
#if CH_OPTIMIZE_SPEED || defined(__DOXYGEN__)
#define port_lock() {                                                       \
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_KERNEL;                 \
  asm volatile ("msr     BASEPRI, %0" : : "r" (tmp) : "memory");            \
}
#else /* !CH_OPTIMIZE_SPEED */
#define port_lock() {                                                       \
  asm volatile ("bl      _port_lock" : : : "r3", "lr", "memory");           \
}
#endif /* !CH_OPTIMIZE_SPEED */
#else /* CORTEX_SIMPLIFIED_PRIORITY */
#define port_lock() asm volatile ("cpsid   i" : : : "memory")
#endif /* CORTEX_SIMPLIFIED_PRIORITY */

/**
 * @brief   Kernel-unlock action.
 * @details Usually this function just disables interrupts but may perform
 *          more actions.
 * @note    In this port this it lowers the base priority to user level.
 */
#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
#if CH_OPTIMIZE_SPEED || defined(__DOXYGEN__)
#define port_unlock() {                                                     \
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_DISABLED;               \
  asm volatile ("msr     BASEPRI, %0" : : "r" (tmp) : "memory");            \
}
#else /* !CH_OPTIMIZE_SPEED */
#define port_unlock() {                                                     \
  asm volatile ("bl      _port_unlock" : : : "r3", "lr", "memory");         \
}
#endif /* !CH_OPTIMIZE_SPEED */
#else /* CORTEX_SIMPLIFIED_PRIORITY */
#define port_unlock() asm volatile ("cpsie   i" : : : "memory")
#endif /* CORTEX_SIMPLIFIED_PRIORITY */

/**
 * @brief   Kernel-lock action from an interrupt handler.
 * @details This function is invoked before invoking I-class APIs from
 *          interrupt handlers. The implementation is architecture dependent,
 *          in its simplest form it is void.
 * @note    Same as @p port_lock() in this port.
 */
#define port_lock_from_isr() port_lock()

/**
 * @brief   Kernel-unlock action from an interrupt handler.
 * @details This function is invoked after invoking I-class APIs from interrupt
 *          handlers. The implementation is architecture dependent, in its
 *          simplest form it is void.
 * @note    Same as @p port_unlock() in this port.
 */
#define port_unlock_from_isr() port_unlock()

/**
 * @brief   Disables all the interrupt sources.
 * @note    Of course non maskable interrupt sources are not included.
 * @note    In this port it disables all the interrupt sources by raising
 *          the priority mask to level 0.
 */
#define port_disable() asm volatile ("cpsid   i" : : : "memory")

/**
 * @brief   Disables the interrupt sources below kernel-level priority.
 * @note    Interrupt sources above kernel level remains enabled.
 * @note    In this port it raises/lowers the base priority to kernel level.
 */
#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
#define port_suspend() {                                                    \
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_KERNEL;                 \
  asm volatile ("msr     BASEPRI, %0                    \n\t"               \
                "cpsie   i" : : "r" (tmp) : "memory");                      \
}
#else /* CORTEX_SIMPLIFIED_PRIORITY */
#define port_suspend() asm volatile ("cpsid   i" : : : "memory")
#endif /* CORTEX_SIMPLIFIED_PRIORITY */

/**
 * @brief   Enables all the interrupt sources.
 * @note    In this port it lowers the base priority to user level.
 */
#if !CORTEX_SIMPLIFIED_PRIORITY || defined(__DOXYGEN__)
#define port_enable() {                                                     \
  register uint32_t tmp asm ("r3") = CORTEX_BASEPRI_DISABLED;               \
  asm volatile ("msr     BASEPRI, %0                    \n\t"               \
                "cpsie   i" : : "r" (tmp) : "memory");                      \
}
#else /* CORTEX_SIMPLIFIED_PRIORITY */
#define port_enable() asm volatile ("cpsie   i" : : : "memory")
#endif /* CORTEX_SIMPLIFIED_PRIORITY */

/**
 * @brief   Enters an architecture-dependent IRQ-waiting mode.
 * @details The function is meant to return when an interrupt becomes pending.
 *          The simplest implementation is an empty function or macro but this
 *          would not take advantage of architecture-specific power saving
 *          modes.
 * @note    Implemented as an inlined @p WFI instruction.
 */
#if CORTEX_ENABLE_WFI_IDLE || defined(__DOXYGEN__)
#define port_wait_for_interrupt() {                                         \
  asm volatile ("wfi" : : : "memory");                                      \
}
#else
#define port_wait_for_interrupt()
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void port_halt(void);
  void port_switch(Thread *ntp, Thread *otp);
  void _port_irq_epilogue(void);
  void _port_switch_from_isr(void);
  void _port_thread_start(void);
#if !CH_OPTIMIZE_SPEED
  void _port_lock(void);
  void _port_unlock(void);
#endif
#ifdef __cplusplus
}
#endif

#endif /* _FROM_ASM_ */

#endif /* _CHCORE_V7M_H_ */

/** @} */