#include "world.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <ucontext.h>

/* Reuse the unit-test rings and wrap their polling operations with waits. */
#define tk_snd_mbf mbf_fake_send
#define tk_rcv_mbf mbf_fake_receive
#include <tk/mbf_fake.h>
#undef tk_snd_mbf
#undef tk_rcv_mbf

#include "comms/telemetry/cmd.h"
#include "comms/telemetry/connection.h"
#include "comms/telemetry/drv_wifi.h"
#include "comms/telemetry/drv_wifi_link.h"
#include "comms/telemetry/mqtt_session.h"
#include "comms/telemetry/net_queue.h"
#include "comms/telemetry/wire_schema.h"
#include "imu/drv_i2c.h"
#include "imu/drv_lsm303.h"
#include "ir/line/track.h"
#include "motion/drv_pwm.h"
#include "motion/encoder.h"
#include "obstacle/drv_sonar.h"
#include "obstacle/obstacle_flags.h"
#include "obstacle_config.h"
#include "obstacle/obstacle_pack.h"
#include "platform/chassis.h"
#include "platform/drv_gpio_irq.h"
#include "platform/drv_watchdog.h"
#include "platform/log.h"
#include "platform/stack.h"
#include "obstacle_internal.h"
#include <tm/tmonitor.h>

#define M_PI_VALUE   (3.14159265358979323846)
#define MAX_TASKS    (16)
#define MAX_CYCLICS  (8)
#define TASK_STACK   (512U * 1024U)
#define MAX_FLAGS    (8)
#define US_PER_MS    (1000U)
#define RUN_ID       (0x0707U)
#define COMMAND_SLOTS (8U)

INT usermain (void);

INT g_shim_prc = (INT)TP_PRC1;
int g_shim_int_enabled;
integration_irq_handler_t g_shim_inthdr;

world_t g_world;

/* Wheel plant: duty to speed, first order. */
#define GAIN_UM_S_PER_PERMILLE (700.0)
#define TAU_MS                 (40.0)
#define UM_PER_MM              (1000.0)
#define MG_PER_MM_S2           (1.0 / 9.80665)
#define SONAR_FORWARD_MM       ((double)CHASSIS_SONAR_FORWARD_MM)
#define SONAR_RISE_US          (300U)

typedef enum
{
    T_DORMANT,
    T_READY,
    T_SLEEP,
    T_FLAG,
    T_MBF,
    T_DONE
} task_state_t;

typedef struct
{
    bool         used;
    task_state_t state;
    void         (*entry)(INT, void *);
    void *       exinf;
    PRI          pri;
    INT          code;
    ucontext_t   ctx;
    void *       stack;
    int          wakeups;
    uint64_t     wake_us;
    bool         timed;
    ER           result;
    ID           flag;
    UINT         wait_ptn;
    UINT         wait_mode;
    UINT *       p_ptn_out;
    ID           mbf;
    void *       p_msg_out;
} task_t;

typedef struct
{
    bool   used;
    UINT   ptn;
} flag_t;

typedef struct
{
    bool     used;
    bool     running;
    FP       handler;
    void *   exinf;
    uint32_t period_ms;
    uint64_t next_us;
} cyclic_t;

static task_t   g_tasks[MAX_TASKS];
static flag_t   g_flags[MAX_FLAGS];
static cyclic_t g_cyclics[MAX_CYCLICS];
static ucontext_t g_sched_ctx;
static int      g_current = -1;

/* Sonar echo of the last trigger. */
static uint64_t g_trig_us;
static uint64_t g_rise_us;
static uint64_t g_fall_us;
static uint32_t g_width_us;
static bool     g_echo_flagged;
static ID       g_echo_flag;

static ID       g_ir_flag;
static ID       g_connection_task;
static uint8_t  g_cmd_queue[COMMAND_SLOTS][TELEMETRY_CMD_WIRE_SIZE];
static unsigned g_cmd_head;
static unsigned g_cmd_count;
static uint8_t  g_run_token_flags;
static uint8_t  g_run_token_resumes;
static bool     g_run_token_valid;
static double   g_wheel_frac[WORLD_WHEELS];
static quadrature_counts_t g_counts[WORLD_WHEELS];
static double   g_last_avg_um_s;
static double   g_accel_filt_mm_s2;
static bool     g_logging;
static uint32_t g_session_car_id;

/* ---- log ---------------------------------------------------------------- */

void log_init (void)
{
    g_logging = (NULL != getenv("INTEGRATION_LOG"));
}

void log_write (log_sev_t sev, log_task_t task, log_mod_t mod, char const * p_fmt, ...)
{
    if (g_logging)
    {
        va_list args;

        (void)fprintf(stderr, "%8u [%d](%d:%d) ", (unsigned)(g_world.now_us / US_PER_MS), (int)sev,
                      (int)task, (int)mod);
        va_start(args, p_fmt);
        (void)vfprintf(stderr, p_fmt, args);
        va_end(args);
        (void)fputc('\n', stderr);
    }
}

void log_drain (void)
{
}

void log_flush (void)
{
}

uint32_t log_dropped (void)
{
    return 0U;
}

void tm_printf (const UB * p_format, ...)
{
    (void)p_format;
}

/* ---- clock and registers ------------------------------------------------ */

uint32_t world_now_ms (void)
{
    return (uint32_t)(g_world.now_us / US_PER_MS);
}

uint32_t shim_reg_read (uint32_t addr)
{
    if (TIMER_TIMERAWL == addr)
    {
        return (uint32_t)g_world.now_us;
    }
    if (TIMER_TIMERAWH == addr)
    {
        return (uint32_t)(g_world.now_us >> 32U);
    }
    return 0U;
}

void shim_reg_write (uint32_t addr, uint32_t value)
{
    (void)addr;
    (void)value;
}

ER tk_get_otm (SYSTIM * p_time)
{
    uint64_t const ms = g_world.now_us / US_PER_MS;

    p_time->hi = (int32_t)(ms >> 32U);
    p_time->lo = (uint32_t)ms;
    return E_OK;
}

/* ---- kernel ------------------------------------------------------------- */

static void task_trampoline (void)
{
    task_t * const p_task = &g_tasks[g_current];

    p_task->entry(p_task->code, p_task->exinf);
    p_task->state = T_DONE;
}

static void block_current (void)
{
    task_t * const p_task = &g_tasks[g_current];

    (void)swapcontext(&p_task->ctx, &g_sched_ctx);
}

ID tk_get_tid (void)
{
    return (g_current >= 0) ? (ID)(g_current + 1) : 1;
}

ID tk_cre_tsk (const T_CTSK * p_task)
{
    for (int idx = 0; idx < MAX_TASKS; idx++)
    {
        if (!g_tasks[idx].used)
        {
            g_tasks[idx] = (task_t){.used = true, .state = T_DORMANT,
                                     .entry = p_task->task, .exinf = p_task->exinf,
                                     .pri = p_task->itskpri};
            return (ID)(idx + 1);
        }
    }
    return E_LIMIT;
}

static void task_start (int idx, INT code)
{
    task_t * const p_task = &g_tasks[idx];

    p_task->stack = mmap(NULL, TASK_STACK, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    p_task->code = code;
    if ((MAP_FAILED == p_task->stack) || (0 != getcontext(&p_task->ctx)))
    {
        perror("integration task context");
        abort();
    }
    p_task->ctx.uc_stack.ss_sp = p_task->stack;
    p_task->ctx.uc_stack.ss_size = TASK_STACK;
    p_task->ctx.uc_link = &g_sched_ctx;
    makecontext(&p_task->ctx, task_trampoline, 0);
    p_task->state = T_READY;
}

ER tk_sta_tsk (ID task, INT code)
{
    if ((task < 1) || (task > MAX_TASKS) || !g_tasks[task - 1].used || (T_DORMANT != g_tasks[task - 1].state))
    {
        return E_OBJ;
    }
    task_start(task - 1, code);
    return E_OK;
}

ER tk_exd_tsk (void)
{
    g_tasks[g_current].state = T_DONE;
    block_current();
    return E_OK;
}

ER tk_slp_tsk (TMO timeout)
{
    task_t * const p_task = &g_tasks[g_current];

    if (p_task->wakeups > 0)
    {
        p_task->wakeups--;
        return E_OK;
    }
    if (TMO_POL == timeout)
    {
        return E_TMOUT;
    }
    p_task->state = T_SLEEP;
    p_task->timed = (TMO_FEVR != timeout);
    p_task->wake_us = g_world.now_us + ((uint64_t)(uint32_t)timeout * US_PER_MS);
    block_current();
    return p_task->result;
}

ER tk_wup_tsk (ID task)
{
    task_t * p_task = NULL;

    if ((task < 1) || (task > MAX_TASKS) || !g_tasks[task - 1].used)
    {
        return E_NOEXS;
    }
    p_task = &g_tasks[task - 1];
    if (T_SLEEP == p_task->state)
    {
        p_task->state = T_READY;
        p_task->result = E_OK;
    }
    else
    {
        p_task->wakeups++;
    }
    return E_OK;
}

INT tk_can_wup (ID task)
{
    task_t * const p_task = &g_tasks[(0 == task) ? g_current : (task - 1)];
    INT const      count = p_task->wakeups;

    p_task->wakeups = 0;
    return count;
}

ID tk_cre_cyc (const T_CCYC * p_config)
{
    for (int idx = 0; idx < MAX_CYCLICS; idx++)
    {
        if (!g_cyclics[idx].used)
        {
            g_cyclics[idx] = (cyclic_t){.used = true,
                                        .running = (0U != (p_config->cycatr & TA_STA)),
                                        .handler = p_config->cychdr,
                                        .exinf = p_config->exinf,
                                        .period_ms = p_config->cyctim,
                                        .next_us = g_world.now_us + ((uint64_t)p_config->cycphs * US_PER_MS)};
            return (ID)(idx + 1);
        }
    }
    return E_LIMIT;
}

ER tk_del_cyc (ID cyclic)
{
    g_cyclics[cyclic - 1].used = false;
    return E_OK;
}

ID tk_cre_flg (const T_CFLG * p_flag)
{
    for (int idx = 0; idx < MAX_FLAGS; idx++)
    {
        if (!g_flags[idx].used)
        {
            g_flags[idx] = (flag_t){.used = true, .ptn = p_flag->iflgptn};
            return (ID)(idx + 1);
        }
    }
    return E_LIMIT;
}

static bool flag_match (UINT ptn, UINT wait_ptn, UINT mode)
{
    return (0U != (mode & TWF_ORW)) ? (0U != (ptn & wait_ptn)) : ((ptn & wait_ptn) == wait_ptn);
}

static void flag_consume (flag_t * p_flag, UINT wait_ptn, UINT mode)
{
    if (0U != (mode & TWF_BITCLR))
    {
        p_flag->ptn &= ~wait_ptn;
    }
}

ER tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn, TMO tmout)
{
    flag_t * const p_flag = &g_flags[flgid - 1];
    task_t * const p_task = &g_tasks[g_current];

    if (flag_match(p_flag->ptn, waiptn, wfmode))
    {
        *p_flgptn = p_flag->ptn;
        flag_consume(p_flag, waiptn, wfmode);
        return E_OK;
    }
    if (TMO_POL == tmout)
    {
        return E_TMOUT;
    }
    p_task->state = T_FLAG;
    p_task->flag = flgid;
    p_task->wait_ptn = waiptn;
    p_task->wait_mode = wfmode;
    p_task->p_ptn_out = p_flgptn;
    p_task->timed = (TMO_FEVR != tmout);
    p_task->wake_us = g_world.now_us + ((uint64_t)(uint32_t)tmout * US_PER_MS);
    block_current();
    return p_task->result;
}

ER tk_set_flg (ID flgid, UINT setptn)
{
    flag_t * const p_flag = &g_flags[flgid - 1];

    p_flag->ptn |= setptn;
    for (int idx = 0; idx < MAX_TASKS; idx++)
    {
        task_t * const p_task = &g_tasks[idx];

        if ((T_FLAG == p_task->state) && (p_task->flag == flgid) &&
            flag_match(p_flag->ptn, p_task->wait_ptn, p_task->wait_mode))
        {
            *p_task->p_ptn_out = p_flag->ptn;
            flag_consume(p_flag, p_task->wait_ptn, p_task->wait_mode);
            p_task->state = T_READY;
            p_task->result = E_OK;
        }
    }
    return E_OK;
}

ER tk_clr_flg (ID flgid, UINT clrptn)
{
    g_flags[flgid - 1].ptn &= clrptn;
    return E_OK;
}

INT tk_rcv_mbf (ID mbf, void * p_msg, TMO timeout)
{
    INT const result = mbf_fake_receive(mbf, p_msg, TMO_POL);

    if ((E_TMOUT != result) || (TMO_POL == timeout))
    {
        return result;
    }
    task_t * const p_task = &g_tasks[g_current];

    p_task->state = T_MBF;
    p_task->mbf = mbf;
    p_task->p_msg_out = p_msg;
    p_task->timed = (TMO_FEVR != timeout);
    p_task->wake_us = g_world.now_us + ((uint64_t)(uint32_t)timeout * US_PER_MS);
    block_current();
    return p_task->result;
}

ER tk_snd_mbf (ID mbf, const void * p_msg, INT msg_size, TMO timeout)
{
    ER const result = mbf_fake_send(mbf, p_msg, msg_size, timeout);

    if (E_OK == result)
    {
        /* Each telemetry queue has one consumer. Give a waiting consumer
         * its message before it returns to the ready list. */
        for (int idx = 0; idx < MAX_TASKS; idx++)
        {
            task_t * const p_task = &g_tasks[idx];

            if ((T_MBF == p_task->state) && (mbf == p_task->mbf))
            {
                p_task->result = mbf_fake_receive(mbf, p_task->p_msg_out, TMO_POL);
                p_task->state = T_READY;
                break;
            }
        }
    }
    return result;
}

static void run_ready (void)
{
    for (;;)
    {
        int best = -1;

        for (int idx = 0; idx < MAX_TASKS; idx++)
        {
            if (g_tasks[idx].used && (T_READY == g_tasks[idx].state) &&
                ((best < 0) || (g_tasks[idx].pri < g_tasks[best].pri)))
            {
                best = idx;
            }
        }
        if (best < 0)
        {
            return;
        }
        g_current = best;
        (void)swapcontext(&g_sched_ctx, &g_tasks[best].ctx);
        g_current = -1;
    }
}

static void fire_timers (void)
{
    for (int idx = 0; idx < MAX_CYCLICS; idx++)
    {
        cyclic_t * const p_cyc = &g_cyclics[idx];

        if (p_cyc->used && p_cyc->running && (g_world.now_us >= p_cyc->next_us))
        {
            p_cyc->next_us += (uint64_t)p_cyc->period_ms * US_PER_MS;
            ((void (*)(void *))p_cyc->handler)(p_cyc->exinf);
        }
    }
    for (int idx = 0; idx < MAX_TASKS; idx++)
    {
        task_t * const p_task = &g_tasks[idx];

        if (((T_SLEEP == p_task->state) || (T_FLAG == p_task->state) ||
             (T_MBF == p_task->state)) && p_task->timed &&
            (g_world.now_us >= p_task->wake_us))
        {
            p_task->state = T_READY;
            p_task->result = E_TMOUT;
        }
    }
}

/* ---- motor bridge ------------------------------------------------------- */

void drv_pwm_boot_brake (void)
{
    g_world.duty[0] = 0;
    g_world.duty[1] = 0;
    g_world.braked = true;
}

ER drv_pwm_init (void)
{
    return E_OK;
}

ER drv_pwm_drive (unsigned int wheel, bool channel_a, uint16_t duty_permille)
{
    (void)wheel;
    (void)channel_a;
    (void)duty_permille;
    return E_OK;
}

ER drv_pwm_set_pair (int16_t left, int16_t right)
{
    if (g_world.latched)
    {
        return E_BUSY;
    }
    if (g_world.pwm_busy)
    {
        return E_BUSY;
    }
    g_world.duty[0] = left;
    g_world.duty[1] = right;
    g_world.braked = false;
    g_world.pwm_writes++;
    return E_OK;
}

ER drv_pwm_brake (void)
{
    g_world.duty[0] = 0;
    g_world.duty[1] = 0;
    g_world.braked = true;
    return E_OK;
}

ER drv_pwm_latch_stop (void)
{
    g_world.duty[0] = 0;
    g_world.duty[1] = 0;
    g_world.braked = true;
    g_world.latched = true;
    return E_OK;
}

/* ---- encoders ----------------------------------------------------------- */

ER encoder_init (void)
{
    return E_OK;
}

ER encoder_reset (void)
{
    return E_OK;
}

ER encoder_read (encoder_sample_t * p_sample)
{
    p_sample->timestamp_us = (uint32_t)g_world.now_us;
    p_sample->wheel[0] = g_counts[0];
    p_sample->wheel[1] = g_counts[1];
    return E_OK;
}

/* ---- gpio and interrupts ------------------------------------------------ */

ER gpio_set_pin (UINT pin, UINT mode)
{
    (void)pin;
    (void)mode;
    return E_OK;
}

UINT gpio_get_val (UINT pin)
{
    if (20U == pin)
    {
        return g_world.start_pin_pressed ? 0U : 1U;
    }
    if (21U == pin)
    {
        return g_world.stop_pin_pressed ? 0U : 1U;
    }
    return 1U;
}

UW GPIO (UINT pin)
{
    return pin;
}

void clr_w (UW address, UW bits)
{
    (void)address;
    (void)bits;
}

ER drv_gpio_irq_init (void)
{
    return E_OK;
}

ER drv_gpio_irq_add_handler (UW proc, uint32_t gpio_mask, gpio_irq_handler_t handler)
{
    (void)proc;
    (void)gpio_mask;
    (void)handler;
    return E_OK;
}

/* ---- watchdog ----------------------------------------------------------- */

uint8_t platform_reset_reason (void)
{
    return PLATFORM_RESET_POWER;
}

bool platform_reset_was_power_up (void)
{
    return true;
}

void platform_wdt_feed (void)
{
    g_world.wdt_feeds++;
}

void platform_run_token_write (uint8_t flags, uint8_t resumes)
{
    g_run_token_flags = flags;
    g_run_token_resumes = resumes;
    g_run_token_valid = true;
}

bool platform_run_token_read (uint8_t * p_flags, uint8_t * p_resumes)
{
    *p_flags = g_run_token_flags;
    *p_resumes = g_run_token_resumes;
    return g_run_token_valid;
}

/* ---- line sensors and barcode ------------------------------------------- */

ER ir_capture_init (void)
{
    T_CFLG const cflg = {.flgatr = TA_WMUL | TA_TFIFO};

    g_ir_flag = tk_cre_flg(&cflg);
    return E_OK;
}

static bool sensor_on_tape (double lateral_mm)
{
    double const lead = (double)CHASSIS_LINE_SENSOR_LEAD_MM;
    double const sx = g_world.x_mm + (lead * cos(g_world.heading_rad)) - (lateral_mm * sin(g_world.heading_rad));
    double const sy = g_world.y_mm + (lead * sin(g_world.heading_rad)) + (lateral_mm * cos(g_world.heading_rad));

    if ((sx < 0.0) || (sx > g_world.tape_end_mm) || ((sx >= g_world.gap_from_mm) && (sx < g_world.gap_to_mm)))
    {
        return false;
    }
    return fabs(sy) <= 7.5;
}

static ir_sensors_state_t ir_state_now (void)
{
    ir_sensors_state_t state = {0};
    double const       half = (double)CHASSIS_LINE_SENSOR_SPACING_MM / 2.0;

    state.left = g_world.ir_forced ? g_world.ir_left : sensor_on_tape(half);
    state.right = g_world.ir_forced ? g_world.ir_right : sensor_on_tape(-half);
    return state;
}

ir_sensors_state_t ir_get_state (void)
{
    return ir_state_now();
}

line_sample_t ir_get_line_sample (void)
{
    return (line_sample_t){.timestamp_us = (uint32_t)g_world.now_us, .state = ir_state_now()};
}

ER ir_wait_change (uint8_t type, TMO tmout)
{
    UINT ptn = 0U;

    return tk_wai_flg(g_ir_flag, 1U << type, TWF_ORW | TWF_BITCLR, &ptn, tmout);
}

uint32_t ir_line_edges (void)
{
    return g_world.ir_edges;
}

uint32_t ir_line_raw_edges (void)
{
    return g_world.ir_edges;
}

uint32_t ir_line_overruns (void)
{
    return 0U;
}

ir_barcode_read_t ir_barcode_pop (uint32_t * p_timestamp_us, bool * p_level)
{
    (void)p_timestamp_us;
    (void)p_level;
    return IR_BARCODE_EMPTY;
}

uint32_t ir_barcode_drops (void)
{
    return 0U;
}

bool ir_barcode_recent (void)
{
    return false;
}

/* ---- sonar and servo ---------------------------------------------------- */

ER scan_hw_init (ID h_echo_flag)
{
    g_echo_flag = h_echo_flag;
    return E_OK;
}

void scan_hw_servo_set (int16_t deci)
{
    g_world.servo_cmd_deci = deci;
    g_world.servo_cmds++;
    if (!g_world.servo_stuck)
    {
        g_world.servo_deci = deci;
    }
}

uint32_t scan_hw_now_us (void)
{
    return (uint32_t)g_world.now_us;
}

bool scan_hw_rearm (void)
{
    g_fall_us = 0U;
    g_rise_us = 0U;
    g_echo_flagged = false;
    return SONAR_STUCK == g_world.sonar_mode;
}

static double ray_range_mm (void)
{
    double const bearing = g_world.heading_rad + ((double)g_world.servo_deci / 10.0) * (M_PI_VALUE / 180.0);
    double const ox = g_world.x_mm + (SONAR_FORWARD_MM * cos(g_world.heading_rad));
    double const oy = g_world.y_mm + (SONAR_FORWARD_MM * sin(g_world.heading_rad));
    double const dx = cos(bearing);
    double const dy = sin(bearing);
    double       best = -1.0;

    for (unsigned idx = 0U; idx < g_world.box_count; idx++)
    {
        world_box_t const * const p_box = &g_world.boxes[idx];
        double t_min = 0.0;
        double t_max = 1.0e9;
        double const o[2] = {ox, oy};
        double const d[2] = {dx, dy};
        double const lo[2] = {p_box->x0_mm, p_box->y0_mm};
        double const hi[2] = {p_box->x1_mm, p_box->y1_mm};
        bool hit = true;

        for (int axis = 0; axis < 2; axis++)
        {
            if (fabs(d[axis]) < 1.0e-9)
            {
                if ((o[axis] < lo[axis]) || (o[axis] > hi[axis]))
                {
                    hit = false;
                }
            }
            else
            {
                double t0 = (lo[axis] - o[axis]) / d[axis];
                double t1 = (hi[axis] - o[axis]) / d[axis];

                if (t0 > t1)
                {
                    double const swap = t0;

                    t0 = t1;
                    t1 = swap;
                }
                t_min = (t0 > t_min) ? t0 : t_min;
                t_max = (t1 < t_max) ? t1 : t_max;
            }
        }
        if (hit && (t_min <= t_max) && ((best < 0.0) || (t_min < best)))
        {
            best = t_min;
        }
    }
    return best;
}

bool scan_hw_trigger (void)
{
    double const range_mm = ray_range_mm();
    uint32_t width = 0U;
    bool echo = true;

    g_world.pings++;
    g_trig_us = g_world.now_us;
    switch (g_world.sonar_mode)
    {
        case SONAR_NO_ECHO:
        case SONAR_STUCK:
            echo = false;
            break;
        case SONAR_GLITCH:
            width = OBSTACLE_GLITCH_MAX_US / 2U;
            break;
        case SONAR_TOO_CLOSE:
            width = (OBSTACLE_GLITCH_MAX_US + OBSTACLE_VALID_MIN_US) / 2U;
            break;
        case SONAR_AUTO:
        default:
            width = (range_mm < 0.0) ? (OBSTACLE_VALID_MAX_US + 500U)
                                      : (uint32_t)((range_mm * (double)OBSTACLE_SOUND_DEN) / (double)OBSTACLE_SOUND_NUM + 1.0);
            break;
    }
    if (echo)
    {
        g_width_us = width;
        g_rise_us = g_trig_us + SONAR_RISE_US;
        g_fall_us = g_rise_us + width;
        g_echo_flagged = false;
    }
    return true;
}

uint32_t scan_hw_echo_result (uint32_t trigger_us)
{
    (void)trigger_us;
    if ((0U != g_fall_us) && (g_world.now_us >= g_fall_us))
    {
        return OBSTACLE_ECHO_RISE | OBSTACLE_ECHO_DONE | (g_width_us & OBSTACLE_ECHO_WIDTH_MASK);
    }
    if ((0U != g_rise_us) && (g_world.now_us >= g_rise_us))
    {
        return OBSTACLE_ECHO_RISE;
    }
    return 0U;
}

bool scan_hw_is_echo_high (void)
{
    return (0U != g_rise_us) && (g_world.now_us >= g_rise_us) && !((0U != g_fall_us) && (g_world.now_us >= g_fall_us));
}

void world_add_box (double x0_mm, double y0_mm, double x1_mm, double y1_mm)
{
    if (g_world.box_count >= WORLD_BOXES_MAX)
    {
        abort();
    }
    g_world.boxes[g_world.box_count++] = (world_box_t){x0_mm, y0_mm, x1_mm, y1_mm};
}

/* ---- IMU ---------------------------------------------------------------- */

void i2c_rp2040_init (i2c_bus_t * p_bus);

static uint8_t g_acc_regs[0x40];
static uint8_t g_mag_regs[0x10];

static void put_accel (uint8_t * p_buf, int16_t mg)
{
    uint16_t const raw = (uint16_t)((uint16_t)mg << 4U);

    p_buf[0] = (uint8_t)(raw & 0xFFU);
    p_buf[1] = (uint8_t)(raw >> 8U);
}

static i2c_bus_result_t bus_write_read (void * p_ctx, uint8_t addr7, const uint8_t * p_wr, uint32_t wr_len,
                                        uint8_t * p_rd, uint32_t rd_len)
{
    (void)p_ctx;
    if (g_world.imu_dead)
    {
        return I2C_BUS_NACK;
    }
    if ((LSM303_ACC_ADDR == addr7) && (0U == rd_len) && (2U == wr_len))
    {
        g_acc_regs[p_wr[0] & 0x3FU] = p_wr[1];
        return I2C_BUS_OK;
    }
    if ((LSM303_MAG_ADDR == addr7) && (0U == rd_len) && (2U == wr_len))
    {
        g_mag_regs[p_wr[0] & 0x0FU] = p_wr[1];
        return I2C_BUS_OK;
    }
    if (LSM303_ACC_ADDR == addr7)
    {
        uint8_t const reg = (uint8_t)(p_wr[0] & 0x3FU);

        if (LSM303_OUT_X_L_A == reg)
        {
            for (unsigned axis = 0U; axis < 3U; axis++)
            {
                put_accel(&p_rd[2U * axis], g_world.accel_mg[axis]);
            }
            return I2C_BUS_OK;
        }
        for (uint32_t idx = 0U; idx < rd_len; idx++)
        {
            p_rd[idx] = g_acc_regs[(reg + idx) & 0x3FU];
        }
        return I2C_BUS_OK;
    }
    if (LSM303_MAG_ADDR == addr7)
    {
        uint8_t const reg = p_wr[0];

        if (LSM303_IRA_REG_M == reg)
        {
            p_rd[0] = 0x48U;
            p_rd[1] = 0x34U;
            p_rd[2] = 0x33U;
            return I2C_BUS_OK;
        }
        if (LSM303_OUT_X_H_M == reg)
        {
            int16_t const x = g_world.mag_raw[0];
            int16_t const y = g_world.mag_raw[1];
            int16_t const z = g_world.mag_raw[2];

            p_rd[0] = (uint8_t)((uint16_t)x >> 8U);
            p_rd[1] = (uint8_t)x;
            p_rd[2] = (uint8_t)((uint16_t)z >> 8U);
            p_rd[3] = (uint8_t)z;
            p_rd[4] = (uint8_t)((uint16_t)y >> 8U);
            p_rd[5] = (uint8_t)y;
            return I2C_BUS_OK;
        }
        for (uint32_t idx = 0U; idx < rd_len; idx++)
        {
            p_rd[idx] = g_mag_regs[(reg + idx) & 0x0FU];
        }
        return I2C_BUS_OK;
    }
    return I2C_BUS_NACK;
}

void i2c_rp2040_init (i2c_bus_t * p_bus)
{
    p_bus->p_ctx = NULL;
    p_bus->write_read = bus_write_read;
    p_bus->recover = NULL;
}

uint32_t physics_time_us (void)
{
    return (uint32_t)g_world.now_us;
}

/* ---- Wi-Fi and MQTT ----------------------------------------------------- */

uint16_t drv_wifi_boot_run_id (void)
{
    return RUN_ID;
}

void drv_wifi_read_status (drv_wifi_status_t * p_status)
{
    *p_status = (drv_wifi_status_t){.join_started = 1U, .join_complete = 1U,
                                   .link_up = g_world.link_up ? 1U : 0U,
                                   .netif_mac = {0x02U, 0x00U, 0x12U, 0x34U, 0x56U, 0x78U}};
}

wifi_lease_t wifi_link_lease (void)
{
    return g_world.link_up ? WIFI_LEASE_READY : WIFI_LEASE_PENDING;
}

wifi_join_t wifi_link_join (void)
{
    return WIFI_JOIN_IDLE;
}

int32_t wifi_link_rssi (void)
{
    return -50;
}

int32_t mqtt_session_init (void)
{
    return E_OK;
}

void mqtt_session_start (int32_t task)
{
    g_connection_task = (ID)task;
}

void mqtt_session_set_car_id (uint32_t car_id)
{
    g_session_car_id = car_id;
}

uint32_t mqtt_session_now_ms (void)
{
    return world_now_ms();
}

bool mqtt_session_connected (void)
{
    return g_world.link_up && g_world.mqtt_up && (0U != g_session_car_id);
}

static uint16_t read_u16 (const uint8_t * p)
{
    return (uint16_t)(((uint16_t)p[0] << 8U) | p[1]);
}

static uint32_t read_u32 (const uint8_t * p)
{
    return ((uint32_t)read_u16(p) << 16U) | read_u16(&p[2]);
}

bool mqtt_session_heartbeat (const uint8_t payload[TELEMETRY_HEARTBEAT_WIRE_SIZE])
{
    if (!mqtt_session_connected())
    {
        return false;
    }
    g_world.heartbeats++;
    g_world.heartbeat_uptime_s = read_u32(&payload[TELEMETRY_HEARTBEAT_OFF_UPTIME_S]);
    g_world.heartbeat_car_id = read_u32(&payload[TELEMETRY_HEARTBEAT_OFF_CAR_ID]);
    g_world.heartbeat_run_id = read_u16(&payload[TELEMETRY_HEARTBEAT_OFF_RUN_ID]);
    return true;
}

bool mqtt_session_command (uint8_t payload[TELEMETRY_CMD_WIRE_SIZE], uint16_t * len)
{
    if (0U == g_cmd_count)
    {
        return false;
    }
    (void)memcpy(payload, g_cmd_queue[g_cmd_head], TELEMETRY_CMD_WIRE_SIZE);
    *len = TELEMETRY_CMD_WIRE_SIZE;
    g_cmd_head = (g_cmd_head + 1U) % COMMAND_SLOTS;
    g_cmd_count--;
    return true;
}

uint32_t mqtt_session_drops (void)
{
    return g_world.mqtt_drops;
}

void world_post_frame (uint16_t cmd_id, uint16_t run_id, uint8_t opcode, uint32_t uptime_s)
{
    if (g_cmd_count >= COMMAND_SLOTS)
    {
        abort();
    }
    uint8_t * const p_frame = g_cmd_queue[(g_cmd_head + g_cmd_count) % COMMAND_SLOTS];

    p_frame[0] = (uint8_t)(cmd_id >> 8U);
    p_frame[1] = (uint8_t)cmd_id;
    p_frame[2] = (uint8_t)(run_id >> 8U);
    p_frame[3] = (uint8_t)run_id;
    p_frame[4] = opcode;
    p_frame[5] = (uint8_t)(uptime_s >> 24U);
    p_frame[6] = (uint8_t)(uptime_s >> 16U);
    p_frame[7] = (uint8_t)(uptime_s >> 8U);
    p_frame[8] = (uint8_t)uptime_s;
    g_cmd_count++;
    if (g_connection_task > 0)
    {
        (void)tk_wup_tsk(g_connection_task);
    }
}

uint16_t world_command (uint8_t opcode)
{
    g_world.cmd_id++;
    world_post_frame(g_world.cmd_id, g_world.heartbeat_run_id, opcode, g_world.heartbeat_uptime_s);
    return g_world.cmd_id;
}

static void note_event (const uint8_t * p_packet)
{
    if (g_world.event_count >= WORLD_EVENTS_MAX)
    {
        abort();
    }
    world_event_t * const p_event = &g_world.events[g_world.event_count++];

    p_event->t_ms = world_now_ms();
    p_event->type = p_packet[TELEMETRY_EVENT_OFF_EVENT_TYPE];
    p_event->source = p_packet[TELEMETRY_EVENT_OFF_SOURCE];
    p_event->p1 = (int16_t)read_u16(&p_packet[TELEMETRY_EVENT_OFF_P1]);
    p_event->p2 = read_u16(&p_packet[TELEMETRY_EVENT_OFF_P2]);
    p_event->odometer_mm = read_u32(&p_packet[TELEMETRY_EVENT_OFF_ODOMETER_MM]);
}

static void radio_step (void)
{
    uint8_t frame[TELEMETRY_NET_QUEUE_MAX_MSG];

    if (!mqtt_session_connected())
    {
        return;
    }
    for (;;)
    {
        INT const len = telemetry_net_queue_pop(frame, TMO_POL);

        if (len <= 0)
        {
            break;
        }
        g_world.frames_published++;
        if ((uint8_t)TELEMETRY_TOPIC_STATUS == frame[0])
        {
            g_world.status_frames++;
            g_world.status_wire_size = (uint16_t)(len - 1);
            if (g_world.status_wire_size == sizeof(g_world.status_payload))
            {
                memcpy(g_world.status_payload, &frame[1], g_world.status_wire_size);
            }
        }
        else if ((uint8_t)TELEMETRY_TOPIC_MOTION == frame[0])
        {
            g_world.separate_motion_frames++;
        }
        else if ((uint8_t)TELEMETRY_TOPIC_PHYSICS == frame[0])
        {
            g_world.separate_physics_frames++;
        }
        else if (((uint8_t)TELEMETRY_TOPIC_EVENT == frame[0]) && ((uint32_t)len >= (1U + TELEMETRY_EVENT_WIRE_SIZE)))
        {
            note_event(&frame[1]);
        }
        else if (((uint8_t)TELEMETRY_TOPIC_DIAG == frame[0]) &&
                 ((uint32_t)len >= (1U + TELEMETRY_DIAG_WIRE_SIZE)))
        {
            const uint8_t * const p_payload = &frame[1];

            g_world.diag_alive_mask = read_u16(&p_payload[TELEMETRY_DIAG_OFF_TASK_ALIVE_MASK]);
            g_world.diag_rejected_run = p_payload[TELEMETRY_DIAG_OFF_CMD_REJ_RUN];
            g_world.diag_rejected_age = p_payload[TELEMETRY_DIAG_OFF_CMD_REJ_AGE];
            g_world.diag_rejected_old_id = p_payload[TELEMETRY_DIAG_OFF_CMD_REJ_OLD_ID];
        }
    }
}

const world_event_t * world_find_event (uint8_t source, uint8_t type, unsigned nth)
{
    for (unsigned idx = 0U; idx < g_world.event_count; idx++)
    {
        if ((g_world.events[idx].source == source) && (g_world.events[idx].type == type))
        {
            if (0U == nth)
            {
                return &g_world.events[idx];
            }
            nth--;
        }
    }
    return NULL;
}

/* ---- plant -------------------------------------------------------------- */

static void plant_step (void)
{
    double const dt_s = 0.001;
    double       avg_um_s = 0.0;
    double       dist_um[WORLD_WHEELS];
    ir_sensors_state_t const before = ir_state_now();

    for (unsigned wheel = 0U; wheel < WORLD_WHEELS; wheel++)
    {
        double const target = (g_world.braked || g_world.latched) ? 0.0 : ((double)g_world.duty[wheel] * GAIN_UM_S_PER_PERMILLE);

        g_world.speed_um_s[wheel] += (target - g_world.speed_um_s[wheel]) * (1.0 / TAU_MS);
        if (g_world.wheel_blocked[wheel] || (fabs(g_world.speed_um_s[wheel]) < 1.0))
        {
            g_world.speed_um_s[wheel] = g_world.wheel_blocked[wheel] ? 0.0 : g_world.speed_um_s[wheel];
        }
        dist_um[wheel] = g_world.speed_um_s[wheel] * dt_s;
        g_world.travel_um[wheel] += dist_um[wheel];
        avg_um_s += g_world.speed_um_s[wheel] / 2.0;
        if (!g_world.enc_dead[wheel])
        {
            /* The left encoder is mounted mirrored: forward counts down. */
            double const steps = dist_um[wheel] * (double)CHASSIS_EDGES_PER_REV / (double)CHASSIS_WHEEL_TRAVEL_UM;
            double const signed_steps = (0U == wheel) ? -steps : steps;

            g_wheel_frac[wheel] += signed_steps;
            while (g_wheel_frac[wheel] >= 1.0)
            {
                g_counts[wheel].positive++;
                g_wheel_frac[wheel] -= 1.0;
            }
            while (g_wheel_frac[wheel] <= -1.0)
            {
                g_counts[wheel].negative++;
                g_wheel_frac[wheel] += 1.0;
            }
        }
    }
    {
        double const mid_mm = (dist_um[0] + dist_um[1]) / 2.0 / UM_PER_MM;
        double const dtheta = (dist_um[1] - dist_um[0]) / UM_PER_MM / (double)CHASSIS_WHEEL_TRACK_MM;

        g_world.x_mm += mid_mm * cos(g_world.heading_rad + (dtheta / 2.0));
        g_world.y_mm += mid_mm * sin(g_world.heading_rad + (dtheta / 2.0));
        g_world.heading_rad += dtheta;
    }
    g_world.a_fwd_mm_s2 = ((avg_um_s - g_last_avg_um_s) / UM_PER_MM) / dt_s;
    g_last_avg_um_s = avg_um_s;
    g_accel_filt_mm_s2 += (g_world.a_fwd_mm_s2 - g_accel_filt_mm_s2) * 0.2;
    if ((0 != g_world.duty[0]) || (0 != g_world.duty[1]))
    {
        g_world.nonzero_duty_ms++;
    }

    /* Level, upright car reads +1 g on Z; forward is -Y. A lifted car is in free fall. */
    g_world.accel_mg[0] = 0;
    g_world.accel_mg[1] = (int16_t)(-g_accel_filt_mm_s2 * MG_PER_MM_S2);
    g_world.accel_mg[2] = g_world.lifted ? 0 : 1000;

    {
        ir_sensors_state_t const after = ir_state_now();

        if ((before.left != after.left) || (before.right != after.right))
        {
            g_world.ir_edges++;
            if (g_ir_flag > 0)
            {
                (void)tk_set_flg(g_ir_flag, 1U << IR_TYPE_LINE);
            }
        }
    }
    if ((0U != g_fall_us) && !g_echo_flagged && (g_world.now_us >= g_fall_us) && (g_echo_flag > 0))
    {
        g_echo_flagged = true;
        (void)tk_set_flg(g_echo_flag, OBSTACLE_FLG_ECHO);
    }
    radio_step();
}

bool world_moving (void)
{
    return (fabs(g_world.speed_um_s[0]) > 1000.0) || (fabs(g_world.speed_um_s[1]) > 1000.0);
}

/* ---- driver of the simulation ------------------------------------------- */

static void boot_task (INT code, void * p_context)
{
    (void)code;
    (void)p_context;
    (void)usermain();
}

void world_boot (void)
{
    T_CTSK const init = {.task = boot_task, .itskpri = 1};
    ID     tid = 0;

    g_world.link_up = true;
    g_world.mqtt_up = true;
    g_world.tape_end_mm = 1.0e9;
    g_world.gap_from_mm = 1.0e9;
    g_world.gap_to_mm = 1.0e9;
    g_world.mag_raw[0] = 220;
    g_world.mag_raw[2] = -390;
    g_world.accel_mg[2] = 1000;
    g_world.sonar_mode = SONAR_AUTO;
    g_world.x_mm = 0.0;
    mbf_fake_reset();
    tid = tk_cre_tsk(&init);
    (void)tk_sta_tsk(tid, 0);
    run_ready();
}

void world_run_ms (uint32_t ms)
{
    for (uint32_t step = 0U; step < ms; step++)
    {
        g_world.now_us += US_PER_MS;
        plant_step();
        fire_timers();
        run_ready();
    }
}

bool world_run_until (bool (*p_done)(void), uint32_t max_ms)
{
    for (uint32_t step = 0U; step < max_ms; step++)
    {
        if (p_done())
        {
            return true;
        }
        world_run_ms(1U);
    }
    return p_done();
}
