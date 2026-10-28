#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "ir_capture.h"
#include "../motion/motion_request.h"
#include "../motion/motion_service.h"
#include "../sys/gpio_irq.h"

LOCAL ER setup_motors (void)
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

EXPORT void line_task (INT stacd, void * exinf)
{
    // TODO: gpio and motion init should be decoupled from this
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
    //     tm_printf((UB *)"motion request: %d\n", result);
    //     if (result != E_OK)
    //     {
    //         return;
    //     }
    //     tk_dly_tsk(100);
    // }

    const uint32_t speed     = 40841U;
    const uint32_t poll_ms   = 20U;
    const uint32_t ms_per_us = 1000U;
    while (true)
    {
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
