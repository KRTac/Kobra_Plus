/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "../../inc/MarlinConfig.h"
#include <stdint.h>
#include "../board/startup.h"
#include "../../core/boards.h"

// ------------------------
// Defines
// ------------------------

// STM32 timers may be 16 or 32 bit. Limiting HAL_TIMER_TYPE_MAX to 16 bits
// avoids issues with STM32F0 MCUs, which seem to pause timers if UINT32_MAX
// is written to the register. STM32F4 timers do not manifest this issue,
// even when writing to 16 bit timers.
//
// The range of the timer can be queried at runtime using IS_TIM_32B_COUNTER_INSTANCE.
// This is a more expensive check than a simple compile-time constant, so its
// implementation is deferred until the desire for a 32-bit range outweighs the cost
// of adding a run-time check and HAL_TIMER_TYPE_MAX is refactored to allow unique
// values for each timer.
#define hal_timer_t uint32_t
#define HAL_TIMER_TYPE_MAX UINT16_MAX

#define HAL_TIMER_RATE uint32_t(F_CPU)  // frequency of timers peripherals

// Marlin timer_instance[] content (unrelated to timer selection)
#define MF_TIMER_STEP       2  // Timer Index for Stepper
#define MF_TIMER_TEMP       1  // Timer Index for Temperature
#define MF_TIMER_PULSE      MF_TIMER_STEP

#define TIMER_INDEX_(T) TIMER##T##_INDEX  // TIMER#_INDEX enums (timer_index_t) depend on TIM#_BASE defines.
#define TIMER_INDEX(T) TIMER_INDEX_(T)    // Convert Timer ID to HardwareTimer_Handle index.

#define TEMP_TIMER_FREQUENCY 1000   // Temperature::isr() is expected to be called at around 1kHz

// TODO: get rid of manual rate/prescale/ticks/cycles taken for procedures in stepper.cpp
#define STEPPER_TIMER_RATE 2000000 // 2 Mhz
extern uint32_t GetStepperTimerClkFreq();
#define STEPPER_TIMER_PRESCALE (GetStepperTimerClkFreq() / (STEPPER_TIMER_RATE))
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs

#define PULSE_TIMER_RATE STEPPER_TIMER_RATE
#define PULSE_TIMER_PRESCALE STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() timer_enable_irq(MF_TIMER_STEP, Enable)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() timer_enable_irq(MF_TIMER_STEP, Disable)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(MF_TIMER_STEP)

#define ENABLE_TEMPERATURE_INTERRUPT() timer_enable_irq(MF_TIMER_TEMP, Enable)
#define DISABLE_TEMPERATURE_INTERRUPT() timer_enable_irq(MF_TIMER_TEMP, Disable)

extern void Step_Handler();
extern void Temp_Handler();

#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() void timer42_zero_match_irq_cb(void)
#endif
#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() void timer41_zero_match_irq_cb(void)
#endif
#define HAL_TONE_TIMER_ISR()      void Timer01B_CallBack(void)

// ------------------------
// Public Variables
// ------------------------

extern HardwareTimer *timer_instance[];

// ------------------------
// Public functions
// ------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

// Configure timer priorities for peripherals such as Software Serial or Servos.
// Exposed here to allow all timer priority information to reside in timers.cpp
void SetTimerInterruptPriorities();

// FORCE_INLINE because these are used in performance-critical situations
FORCE_INLINE bool HAL_timer_initialized(const uint8_t timer_num) {
  return timer_instance[timer_num] != nullptr;
}
FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) ? timer_get_count(timer_num) : 0;
}

// NOTE: Method name may be misleading.
// STM32 has an Auto-Reload Register (ARR) as opposed to a "compare" register

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare);

#define HAL_timer_isr_prologue(T) NOOP
//#define HAL_timer_isr_epilogue(T) NOOP

static inline void HAL_timer_isr_epilogue(uint8_t timer_num)
{
    if(timer_num == MF_TIMER_TEMP) {
        TIMER4_CNT_ClearIrqFlag(M4_TMR41, Timer4CntZeroMatchInt);
    } else if(timer_num == MF_TIMER_STEP) {
        TIMER4_CNT_ClearIrqFlag(M4_TMR42, Timer4CntZeroMatchInt);
    }
}

#define TIMER_OC_NO_PRELOAD 0 // Need to disable preload also on compare registers.