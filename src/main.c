#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "demo_tasks.h"
#include "ir/ir_capture.h"
#include "motion/motion_request.h"
#include "motion/motion_service.h"
#include "motion/encoder_capture.h"
#include "sys/gpio_irq.h"

/* Every task uses 4096 bytes.  The SMP kernel path is deeper than the
   single-core one -- each critical section runs the global ready-queue
   assignment on the calling task's stack -- so 1-2 KiB is no longer enough
   for a task that makes blocking kernel calls. */
#define STACK_SZ 4096

ER setup_motors (void)
{
    motion_service_config_t config   = {0};
    config.control.sampling.wheel[0] = (motion_scale_t){204204U, 2530U, -1};
    config.control.sampling.wheel[1] = (motion_scale_t){204204U, 2530U, 1};
    config.control.sampling.max_interval_us = 100000U;
    for (uint8_t wheel = 0U; wheel < 2U; wheel++)
    {
        config.control.speed[wheel] = (wheel_pid_config_t){
            .kp_q16            = 256U,
            .ki_q16            = 256U,
            .kd_q16            = 16U,
            .max_target_um_s   = 100000U,
            .max_interval_us   = 100000U,
            .max_sample_age_us = 600000U,
            .max_duty_permille = 250U,
        };
    }
    config.control.max_command_ttl_us  = 200000U;
    config.control.max_control_gap_us  = 100000U;
    config.control.feedback_timeout_us = 2000000U;
    config.control.reversal_brake_us   = 100000U;
    config.poll_delay_ms               = 10U;

    return motion_service_start(&config);
}

LOCAL void line_task (INT stacd, void * exinf)
{
    ER result = gpio_irq_init();
    if (result != E_OK)
    {
        tm_printf((UB *)"gpio_irq_init failed: %d\n", result);
    }

    (void)stacd;
    (void)exinf;

    result = setup_motors();
    if (result != E_OK)
    {
        tm_printf((UB *)"setup_motors failed: %d\n", result);
        return;
    }
    tm_printf((UB *)"Inited setup_motors\n");

    result = ir_capture_init();
    if (result != E_OK)
    {
        tm_printf((UB *)"ir_capture_init failed: %d\n", result);
        return;
    }
    tm_printf((UB *)"Inited ir_capture\n");

    // for (int i = 0; i < 50; i++) // Counts of 100ms
    // {
    //     motion_drive_request_t request = {
    //         .action       = MOTION_DRIVE_STRAIGHT,
    //         .inside_um_s  = 0U,
    //         .speed_um_s   = 40841U,
    //         .sensed_us    = in_w(TIMER_BASE + 0x28U),
    //         .valid_for_us = 100000U,
    //     };
    //     result = motion_service_submit_request(&request);
    //     tm_printf((UB *)"set speed: %d\n", result);
    //     if (result != E_OK)
    //     {
    //         return;
    //     }
    //     tk_dly_tsk(100);
    // }

    const uint32_t speed     = 40841U;
    const uint32_t poll_ms   = 20;
    const uint32_t ms_per_us = 1000U;
    // int            count     = 0;
    while (true)
    {
        // tm_printf((UB *)"%d\n", count++);

        // encoder_capture_sample_t s;
        // encoder_capture_read(&s);
        // tm_printf((UB *)"L pos: %d, L neg: %d\n", s.wheel[0].positive,
        //           s.wheel[0].negative);
        // tm_printf((UB *)"R pos: %d, R neg: %d\n", s.wheel[1].positive,
        //           s.wheel[1].negative);

        // tm_printf((UB *)"left: %d\n", ir_get_state().left);
        // tm_printf((UB *)"right: %d\n", ir_get_state().right);

        const ir_sensors_state_t state = ir_get_state();
        if (state.left && state.right)
        {
            // TODO: run command at junction
            motion_service_set_speed(0, 0, poll_ms * ms_per_us);
        }
        else if (state.left)
        {
            motion_service_set_speed(0, speed, poll_ms * ms_per_us);
        }
        else if (state.right)
        {
            motion_service_set_speed(speed, 0, poll_ms * ms_per_us);
        }
        else
        {
            motion_service_set_speed(speed, speed, poll_ms * ms_per_us);
        }
        ir_wait_change(IR_TYPE_LINE, poll_ms);
    }
}

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 * ------------------------------------------------------------------ */

LOCAL T_CTSK ctsk_blink = {
    .itskpri = 10,
    .stksz   = STACK_SZ,
    .task    = blink_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

LOCAL T_CTSK ctsk_line = {
    .itskpri = 9,
    .stksz   = STACK_SZ,
    .task    = line_task,
#if TK_SUPPORT_SMP
    .tskatr = TA_HLNG | TA_RNG3 | TA_ASSPRC,
    .assprc = TP_PRC1,
#else
    .tskatr = TA_HLNG | TA_RNG3,
#endif
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or print failure and stop. */
LOCAL BOOL made (const char * what, ID id)
{
    if (id <= E_OK)
    {
        tm_printf((UB *)"[init] tk_cre_%s failed, er=%d\n", what, id);
        return FALSE;
    }
    return TRUE;
}

EXPORT INT usermain (void)
{

    ID tid = tk_cre_tsk(&ctsk_blink);
    if (made("tsk(blink)", tid))
    {
        tk_sta_tsk(tid, 0);
    }
    tid = tk_cre_tsk(&ctsk_line);
    if (made("tsk(line)", tid))
    {
        tk_dly_tsk(3000);
        tk_sta_tsk(tid, 0);
    }

    /* The initial task has nothing left to do.  It must not return: on
       return the kernel shuts the system down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
