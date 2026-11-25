/** @file internal.h
 * @brief Private handoffs between the supervisor task, admission and telemetry.
 */
#ifndef SUPERVISOR_INTERNAL_H
#define SUPERVISOR_INTERNAL_H

#include "run_state.h"
#include "supervisor/supervisor.h"

/* Run-state and mailbox updates owned by the supervisor pass. */
void supervisor_admit_command (vehicle_cmd_t cmd, run_inputs_t in, run_result_t * p_res);
void supervisor_apply_result (const run_result_t * p_res, vehicle_cmd_t cmd, bool b_command);

void supervisor_admission_init (const supervisor_config_t * p_config);
void supervisor_sync_run_token (bool b_armed, bool b_continuous);
void supervisor_note_start_accepted (void);
void supervisor_selftest (void);
void supervisor_poll_start_button (const supervisor_config_t * p_config);
void supervisor_handle_command (const run_ctx_t * p_ctx, vehicle_cmd_t cmd,
                                const run_inputs_t * p_in, bool b_critical);
void supervisor_finish_calibration_start (const run_ctx_t * p_ctx, const run_inputs_t * p_in,
                                          bool b_critical);
void supervisor_consider_parked_starts (run_ctx_t * p_ctx, const run_inputs_t * p_in,
                                       bool b_permit_renewed, bool b_critical,
                                       const supervisor_config_t * p_config);
bool supervisor_parked_observation (const run_inputs_t * p_in, uint32_t hold,
                                   uint16_t clear_mm, bool * p_clear);
void supervisor_reset_parked_observation (void);

void supervisor_telem_init (void);
void supervisor_emit_event (uint8_t source, uint8_t type, int16_t p1, uint16_t p2);
void supervisor_report_command (vehicle_cmd_t cmd, const run_result_t * p_res);
void supervisor_arm_stop_timing (void);
/* Check the reported episode before commands, and an active one afterwards. */
void supervisor_watch_stop_timing (bool b_estop, bool b_reported);
void supervisor_watch_edges (run_state_t state, bool b_unverified_start);
void supervisor_report_stall (uint8_t id, uint8_t misses);
void supervisor_report_build (void);

#endif /* SUPERVISOR_INTERNAL_H */

/*** end of file ***/
