/*
 *	health_ids.h
 *	Liveness bit numbers (D160). A task includes this to kick its own bit
 *	with liveness_kick() from platform/liveness.h.
 */

#ifndef SUPERVISOR_HEALTH_IDS_H
#define SUPERVISOR_HEALTH_IDS_H

#define LIVE_MOTION     (0U)
#define LIVE_LINE       (1U)
#define LIVE_OBSTACLE   (2U)
#define LIVE_PHYSICS    (3U)
#define LIVE_TELEMETRY  (4U)
#define LIVE_CONNECTION (5U)
#define LIVE_BARCODE    (6U)
#define LIVE_TASKS      (7U)

#endif /* SUPERVISOR_HEALTH_IDS_H */
