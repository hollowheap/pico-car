#ifndef INTEGRATION_WORLD_H
#define INTEGRATION_WORLD_H

/* The simulated car: a cooperative kernel, a clock, and the hardware the
 * firmware drivers would touch. The firmware tasks run unchanged on top. */

#include <stdbool.h>
#include <stdint.h>

#define WORLD_WHEELS      (2U)
#define WORLD_EVENTS_MAX  (4096U)
#define WORLD_BOXES_MAX   (4U)
#define WORLD_CAR_ID     (0x12345678U)

typedef enum
{
    SONAR_AUTO,     /* range the boxes in front of the sensor */
    SONAR_NO_ECHO,  /* the echo line never rises */
    SONAR_STUCK,    /* the echo line is already high at the trigger */
    SONAR_GLITCH,   /* a pulse too short to be a reflection */
    SONAR_TOO_CLOSE /* a clean echo inside the minimum range */
} world_sonar_mode_t;

typedef struct
{
    uint32_t t_ms;
    uint32_t odometer_mm;
    int16_t  p1;
    uint16_t p2;
    uint8_t  source;
    uint8_t  type;
} world_event_t;

typedef struct
{
    double x0_mm;
    double y0_mm;
    double x1_mm;
    double y1_mm;
} world_box_t;

typedef struct
{
    uint64_t now_us;

    /* Motor bridge as the PWM driver last left it. */
    int16_t  duty[WORLD_WHEELS];
    bool     braked;
    bool     latched;
    uint32_t pwm_writes;
    uint32_t nonzero_duty_ms;
    bool     pwm_busy;

    /* Wheels and body. Speeds and travel are car-forward positive. */
    double   speed_um_s[WORLD_WHEELS];
    double   travel_um[WORLD_WHEELS];
    bool     enc_dead[WORLD_WHEELS];
    bool     wheel_blocked[WORLD_WHEELS];
    double   x_mm;
    double   y_mm;
    double   heading_rad;
    double   a_fwd_mm_s2;
    bool     stop_pin_pressed;
    bool     start_pin_pressed;

    /* Line sensors: a straight tape on the x axis. */
    bool     ir_forced;
    bool     ir_left;
    bool     ir_right;
    double   tape_end_mm;
    double   gap_from_mm;
    double   gap_to_mm;
    uint32_t ir_edges;

    /* Sonar and its servo. */
    world_sonar_mode_t sonar_mode;
    world_box_t        boxes[WORLD_BOXES_MAX];
    unsigned           box_count;
    int16_t            servo_cmd_deci;
    int16_t            servo_deci;
    bool               servo_stuck;
    uint32_t           servo_cmds;
    uint32_t           pings;

    /* IMU: milli-g in the sensor frame, and magnetometer counts. */
    bool     lifted;
    bool     imu_dead;
    int16_t  accel_mg[3];
    int16_t  mag_raw[3];

    /* Radio. */
    bool     link_up;
    bool     mqtt_up;
    uint32_t mqtt_drops;
    uint32_t frames_published;
    uint32_t heartbeats;
    uint32_t heartbeat_uptime_s;
    uint32_t heartbeat_car_id;
    uint16_t heartbeat_run_id;
    uint16_t cmd_id;
    uint16_t diag_alive_mask;
    uint8_t  diag_rejected_run;
    uint8_t  diag_rejected_age;
    uint8_t  diag_rejected_old_id;

    /* Watchdog and platform. */
    uint32_t wdt_feeds;

    world_event_t events[WORLD_EVENTS_MAX];
    unsigned      event_count;
} world_t;

extern world_t g_world;

/* Boot as usermain() does, then run until the startup barrier opened. */
void world_boot (void);

void     world_run_ms (uint32_t ms);
bool     world_run_until (bool (*p_done)(void), uint32_t max_ms);
uint32_t world_now_ms (void);

/* Hand one command frame to the firmware the way the MQTT session does. */
void world_post_frame (uint16_t cmd_id, uint16_t run_id, uint8_t opcode, uint32_t uptime_s);
/* A well-formed command using the run and uptime received in the heartbeat. */
uint16_t world_command (uint8_t opcode);

const world_event_t * world_find_event (uint8_t source, uint8_t type, unsigned nth);
void world_add_box (double x0_mm, double y0_mm, double x1_mm, double y1_mm);
bool world_moving (void);

#endif
