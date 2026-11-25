/** @file alarm.h
 *
 * @brief RP2040 hardware timer alarm allocation (D88, S31).
 *
 * RP2040 provides four hardware timer alarms (0..3).
 * Alarm 0 is dedicated to 5 ms line sensor sampling on Core 0.
 * Its corresponding NVIC interrupt vector is TIMER_IRQ_0 (IRQ 0).
 */
#ifndef PLATFORM_ALARM_H
#define PLATFORM_ALARM_H

#define LINE_TIMER_ALARM_NUM (0U)
#define LINE_TIMER_ALARM_IRQ (0U)
#define LINE_TIMER_ALARM_PRI (2U)

#endif /* PLATFORM_ALARM_H */

/*** end of file ***/
