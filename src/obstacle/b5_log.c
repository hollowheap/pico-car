/** @file b5_log.c
 *
 * @brief Buddy 5 flight recorder. Records go into a RAM ring (the oldest
 *        is overwritten when it is full). When the USB serial port is
 *        opened (DTR set), the whole ring is printed, a few lines per call,
 *        so the line task is never held up and the USB ring never overflows.
 *
 * Fields used by each record kind (a..f are val_a..val_f; unused ones are 0):
 *   BOOT    d = obstacle_init() result
 *   STATE   a = Buddy 5 state, b = stop cause, c = fault byte
 *   OBST    d = distance (mm)
 *   FAULT   a = stop cause, c = fault byte
 *   RESULT  a = status, b = side, c = start angle (deg), d = width (mm),
 *           e = profile store index
 *   STEER   a = steer, d = left speed (mm/s), e = right speed (mm/s),
 *           f = heading since the bypass began (deci-degrees, + = left)
 *   MOTION  a = motion state, b = motion fault, c = hold mask
 *   CMD     a = command, b = request id, d = result
 *   TRACE   a = Buddy 5 state, b = echo class, c = IR bits (1 left, 2 right)
 *           plus steer << 2, d = distance (mm), e = left wheel (mm),
 *           f = right wheel (mm)
 *   TURNED  d = heading turned (deci-degrees), e = planned angle (deg)
 *   MFAULT  a = wheel, b = speed_valid, c = odometry_valid, d = bad
 *           encoder steps, e = speed (mm/s), f = sample interval (us)
 *   PHASE   a = bypass phase (0 TURN, 1 OUT, 2 HUG), d =
 * side offset (mm), e = distance along the line (mm), f = heading
 *           (deci-degrees, positive = away from the obstacle side)
 *   GUARD   a = 1 guard on / 0 off, b = bypass phase, d = filtered 45
 *           degree reading (mm), f = heading (deci-degrees, positive =
 *           away from the obstacle side)
 *
 * NOTE: The kernel types UB, INT, UINT and SYSTIM are used where the
 * kernel requires them (deviation from BARR-C 5.2.a, see
 * docs/buddy5_obstacle.md). Casts to (UB const *) and (INT) feed
 * tm_printf() only.
 */

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../usb_console_compat.h"
#include "b5_cfg.h"
#include "b5_log.h"
#include "b5_pack.h"

/* The ring index wraps with a mask, so the size must be a power of two. */
#define LOG_MASK (B5_LOG_RECORDS - 1U)

/* tm_usb_state() value once a serial monitor has opened the port. */
#define USB_OPEN (3U)

/* Name table sizes. */
#define STATE_NAMES  (8U)
#define CAUSE_NAMES  (16U)
#define FIELD2_NAMES (4U)
#define MOTION_NAMES (8U)
#define CMD_NAMES    (8U)
#define PHASE_NAMES  (3U)

/* Slots printed per profile line. */
#define SLOTS_PER_LINE (5U)

/* Deci-degrees per degree. */
#define DECI_PER_DEG (10)

/* IR and steer bits of a TRACE record. */
#define TRACE_IR_LEFT    (0x1U)
#define TRACE_IR_RIGHT   (0x2U)
#define TRACE_STEER_SH   (2U)
#define TRACE_STEER_MASK (0x3U)

/* One stored scan profile. */
typedef struct
{
    uint32_t t_ms;
    uint8_t  req_id;
    uint16_t slot[B5_SLOT_COUNT];
} log_profile_t;

/* Recorder state; only the line task touches it. NOTE: Booleans are b_
 * members of a g_ struct, which satisfies both BARR-C 7.1.m/7.1.o and the
 * team .clang-tidy. */
typedef struct
{
    b5_log_rec_t  rec[B5_LOG_RECORDS];
    log_profile_t profile[B5_LOG_PROFILES];
    uint32_t      head;
    uint32_t      count;
    uint32_t      missed;
    uint32_t      next_profile;
    uint32_t      dump_index;
    uint32_t      dump_total;
    uint32_t      dump_first;
    bool          b_was_open;
    bool          b_dumping;
} log_state_t;

static log_state_t g_log;

static char const * const g_state_name[STATE_NAMES] = {
    "STOPPED", "MONITOR", "WAIT_STOP", "SCANNING",
    "PLAN",    "BYPASS",  "REACQUIRE", "?"};
static char const * const g_cause_name[CAUSE_NAMES]   = {"NONE",
                                                         "BOOT",
                                                         "EXTERNAL",
                                                         "NO_ECHO",
                                                         "ECHO_STUCK",
                                                         "SCAN_TIMEOUT",
                                                         "NO_SAFE_PATH",
                                                         "COLLISION",
                                                         "HANDOVER",
                                                         "PLAN_TIMEOUT",
                                                         "PLAN_REJECT",
                                                         "BYPASS_LIMIT",
                                                         "LINE_TIMEOUT",
                                                         "INIT_FAIL",
                                                         "?",
                                                         "?"};
static char const * const g_class_name[FIELD2_NAMES]  = {"VALID", "OPEN",
                                                         "UNKNOWN", "STUCK"};
static char const * const g_steer_name[FIELD2_NAMES]  = {"STRAIGHT", "LEFT",
                                                         "RIGHT", "HOLD"};
static char const * const g_result_name[FIELD2_NAMES] = {
    "OK", "CLEAR", "NO_SAFE_PATH", "FAILED"};
static char const * const g_side_name[FIELD2_NAMES]   = {"LEFT", "RIGHT", "?",
                                                         "?"};
static char const * const g_mstate_name[MOTION_NAMES] = {
    "IDLE", "RUNNING", "REVERSING", "STOPPED", "FAULT", "?", "?", "?"};
static char const * const g_mfault_name[MOTION_NAMES] = {
    "NONE", "COMMAND", "EXPIRED", "FEEDBACK", "GAP", "NO_EDGES", "OUTPUT", "?"};
static char const * const g_cmd_name[CMD_NAMES] = {
    "NONE",       "SCAN_REQ", "ACCEPT", "REJECT",
    "LINE_FOUND", "STOP",     "RESET",  "START"};

static char const * const g_phase_name[PHASE_NAMES] = {"TURN", "OUT", "HUG"};

static uint32_t now_ms (void);
static void     print_profile (uint32_t index, uint32_t t_ms);
static void     print_record (b5_log_rec_t const * p_rec);
static void     print_nav_record (b5_log_rec_t const * p_rec);
static void     start_dump (void);
static void     continue_dump (void);

/*!
 * @brief Add one record, stamped with the current time. While a dump is
 *        being printed, records are counted as missed instead.
 *
 * @param[in] p_rec Record to add (its t_ms is replaced).
 */
void b5_log_add (b5_log_rec_t const * p_rec)
{
    if (g_log.b_dumping)
    {
        g_log.missed++;
    }
    else
    {
        b5_log_rec_t * const p_slot = &g_log.rec[g_log.head];

        *p_slot      = *p_rec;
        p_slot->t_ms = now_ms();
        g_log.head   = (g_log.head + 1U) & LOG_MASK;

        if (g_log.count < B5_LOG_RECORDS)
        {
            g_log.count++;
        }
    }
}

/*!
 * @brief Store a scan profile. Log a RESULT record with the returned index
 *        straight after, so the dump can print the two together.
 *
 * @param[in] req_id    Request id of the scan.
 * @param[in] p_profile B5_SLOT_COUNT packed slots.
 *
 * @return Store index for the RESULT record's e field.
 */
uint32_t b5_log_profile (uint8_t req_id, uint16_t const * p_profile)
{
    uint32_t const        index   = g_log.next_profile;
    log_profile_t * const p_store = &g_log.profile[index];
    uint32_t              slot    = 0U;

    if (!g_log.b_dumping)
    {
        p_store->t_ms   = now_ms();
        p_store->req_id = req_id;

        for (slot = 0U; slot < B5_SLOT_COUNT; slot++)
        {
            p_store->slot[slot] = p_profile[slot];
        }

        g_log.next_profile = (index + 1U) % B5_LOG_PROFILES;
    }

    return (index);
}

/*!
 * @brief Call every line-task pass: start a dump when the serial port has
 *        just been opened, and print the next few lines of a dump.
 */
void b5_log_service (void)
{
    bool const b_open = (tm_usb_state() >= USB_OPEN);

    if (b_open && (!g_log.b_was_open))
    {
        start_dump();
    }
    else if ((!b_open) && g_log.b_dumping)
    {
        /* The monitor was closed: give up and keep recording. */
        g_log.b_dumping = false;
    }
    else
    {
        /* Nothing new. */
    }

    g_log.b_was_open = b_open;

    if (g_log.b_dumping)
    {
        continue_dump();
    }
}

/*!
 * @brief Kernel time in milliseconds.
 *
 * @return Low 32 bits of the system time.
 */
static uint32_t now_ms (void)
{
    SYSTIM tim = {0, 0U};

    (void)tk_get_otm(&tim);

    return (tim.lo);
}

/*!
 * @brief Begin printing the ring, oldest record first.
 */
static void start_dump (void)
{
    g_log.dump_total = g_log.count;
    g_log.dump_first = (g_log.head - g_log.count) & LOG_MASK;
    g_log.dump_index = 0U;
    g_log.b_dumping  = true;

    tm_printf((UB const *)"\n===== Buddy 5 flight log: %d records, now %d ms,"
                          " %d missed during earlier dumps =====\n",
              (INT)g_log.dump_total, (INT)now_ms(), (INT)g_log.missed);
}

/*!
 * @brief Print the next few records, only while the USB ring has room.
 */
static void continue_dump (void)
{
    uint32_t lines = 0U;

    while (g_log.b_dumping && (lines < B5_LOG_LINES_PER_PASS) &&
           (tm_usb_pending() < B5_LOG_PENDING_MAX))
    {
        if (g_log.dump_index >= g_log.dump_total)
        {
            tm_printf((UB const *)"===== end of Buddy 5 flight log =====\n");
            g_log.b_dumping = false;
        }
        else
        {
            uint32_t const index =
                (g_log.dump_first + g_log.dump_index) & LOG_MASK;

            print_record(&g_log.rec[index]);
            g_log.dump_index++;
        }

        lines++;
    }
}

/*!
 * @brief Print one stored profile, five slots per line.
 *
 * @param[in] index Profile store index from the RESULT record.
 * @param[in] t_ms  Time of the RESULT record, to spot an overwritten one.
 */
static void print_profile (uint32_t index, uint32_t t_ms)
{
    log_profile_t const * const p_store =
        &g_log.profile[index % B5_LOG_PROFILES];
    uint32_t slot = 0U;

    /* The profile is stored just before its RESULT record. */
    if ((t_ms - p_store->t_ms) > B5_LOG_PROFILE_MATCH_MS)
    {
        tm_printf((UB const *)"           (profile overwritten)\n");
    }
    else
    {
        for (slot = 0U; slot < B5_SLOT_COUNT; slot++)
        {
            uint16_t const value = p_store->slot[slot];
            INT const      angle = (INT)(B5_SLOT_FIRST_DECI +
                                    ((int32_t)slot * B5_SLOT_STEP_DECI)) /
                              DECI_PER_DEG;

            if (0U == (slot % SLOTS_PER_LINE))
            {
                tm_printf((UB const *)"          ");
            }

            if (b5_pack_slot_is_measured(value))
            {
                tm_printf((UB const *)" %+3d:%-7s%4d |", angle,
                          g_class_name[b5_pack_slot_class(value)],
                          (INT)b5_pack_slot_dist(value));
            }
            else
            {
                tm_printf((UB const *)" %+3d:--          |", angle);
            }

            if ((SLOTS_PER_LINE - 1U) == (slot % SLOTS_PER_LINE))
            {
                tm_printf((UB const *)"\n");
            }
        }
    }
}

/*!
 * @brief Print one record as a readable line.
 *
 * @param[in] p_rec Record to print.
 */
static void print_record (b5_log_rec_t const * p_rec)
{
    INT const t_ms = (INT)p_rec->t_ms;

    switch (p_rec->kind)
    {
        case B5_LOG_BOOT:
            tm_printf((UB const *)"%8d BOOT obstacle_init=%d\n", t_ms,
                      (INT)p_rec->val_d);
        break;

        case B5_LOG_STATE:
            tm_printf((UB const *)"%8d STATE %s cause=%s faults=0x%02x\n", t_ms,
                      g_state_name[p_rec->val_a % STATE_NAMES],
                      g_cause_name[p_rec->val_b % CAUSE_NAMES],
                      (UINT)p_rec->val_c);
        break;

        case B5_LOG_OBST:
            tm_printf((UB const *)"%8d OBSTACLE at %d mm\n", t_ms,
                      (INT)p_rec->val_d);
        break;

        case B5_LOG_FAULT:
            tm_printf((UB const *)"%8d FAULT %s faults=0x%02x\n", t_ms,
                      g_cause_name[p_rec->val_a % CAUSE_NAMES],
                      (UINT)p_rec->val_c);
        break;

        case B5_LOG_RESULT:
            tm_printf((UB const *)"%8d SCAN_RESULT %s side=%s width=%d mm "
                                  "start=%d deg\n",
                      t_ms, g_result_name[p_rec->val_a % FIELD2_NAMES],
                      g_side_name[p_rec->val_b % FIELD2_NAMES],
                      (INT)p_rec->val_d, (INT)p_rec->val_c);
            print_profile((uint32_t)p_rec->val_e, p_rec->t_ms);
        break;

        case B5_LOG_STEER:
            tm_printf((UB const *)"%8d STEER %s L=%d R=%d mm/s turned=%d deg\n",
                      t_ms, g_steer_name[p_rec->val_a % FIELD2_NAMES],
                      (INT)p_rec->val_d, (INT)p_rec->val_e,
                      (INT)(p_rec->val_f / DECI_PER_DEG));
        break;

        case B5_LOG_MOTION:
            tm_printf((UB const *)"%8d MOTION state=%s fault=%s hold=%d\n",
                      t_ms, g_mstate_name[p_rec->val_a % MOTION_NAMES],
                      g_mfault_name[p_rec->val_b % MOTION_NAMES],
                      (INT)p_rec->val_c);
        break;

        case B5_LOG_CMD:
            tm_printf((UB const *)"%8d CMD %s id=%d -> %d\n", t_ms,
                      g_cmd_name[p_rec->val_a % CMD_NAMES], (INT)p_rec->val_b,
                      (INT)p_rec->val_d);
        break;

        case B5_LOG_TRACE:
            tm_printf((UB const *)"%8d   . %-9s %-7s %4d mm IR=%c%c "
                                  "wheels L=%d R=%d mm %s\n",
                      t_ms, g_state_name[p_rec->val_a % STATE_NAMES],
                      g_class_name[p_rec->val_b % FIELD2_NAMES],
                      (INT)p_rec->val_d,
                      (0U != (p_rec->val_c & TRACE_IR_LEFT)) ? 'L' : '-',
                      (0U != (p_rec->val_c & TRACE_IR_RIGHT)) ? 'R' : '-',
                      (INT)p_rec->val_e, (INT)p_rec->val_f,
                      g_steer_name[((uint32_t)p_rec->val_c >> TRACE_STEER_SH) &
                                   TRACE_STEER_MASK]);
        break;

        default:
            print_nav_record(p_rec);
        break;
    }
}

/*!
 * @brief Print one bypass record (TURNED, PHASE, MFAULT, GUARD) as a
 *        readable line.
 *
 * @param[in] p_rec Record to print.
 */
static void print_nav_record (b5_log_rec_t const * p_rec)
{
    INT const t_ms = (INT)p_rec->t_ms;

    switch (p_rec->kind)
    {
        case B5_LOG_TURNED:
            tm_printf((UB const *)"%8d ESCAPE planned angle reached: turned "
                                  "%d deg (plan %d deg)\n",
                      t_ms, (INT)(p_rec->val_d / DECI_PER_DEG),
                      (INT)p_rec->val_e);
        break;

        case B5_LOG_PHASE:
            tm_printf((UB const *)"%8d PHASE %s: side %d mm, ahead %d mm, "
                                  "heading %d deg\n",
                      t_ms, g_phase_name[p_rec->val_a % PHASE_NAMES],
                      (INT)p_rec->val_d, (INT)p_rec->val_e,
                      (INT)(p_rec->val_f / DECI_PER_DEG));
        break;

        case B5_LOG_MFAULT:
            tm_printf((UB const *)"%8d   MOTION FAULT wheel %d: speed_valid %d "
                                  "odo_valid %d bad steps %d speed %d mm/s "
                                  "interval %d us\n",
                      t_ms, (INT)p_rec->val_a, (INT)p_rec->val_b,
                      (INT)p_rec->val_c, (INT)p_rec->val_d, (INT)p_rec->val_e,
                      (INT)p_rec->val_f);
        break;

        case B5_LOG_GUARD:
            tm_printf((UB const *)"%8d GUARD %s in %s: 45 deg reading %d mm "
                                  "(median of 3), heading %d deg\n",
                      t_ms, (0U != p_rec->val_a) ? "ON" : "off",
                      g_phase_name[p_rec->val_b % PHASE_NAMES],
                      (INT)p_rec->val_d, (INT)(p_rec->val_f / DECI_PER_DEG));
        break;

        default:
            tm_printf((UB const *)"%8d ? kind %d\n", t_ms, (INT)p_rec->kind);
        break;
    }
}

/*** end of file ***/

