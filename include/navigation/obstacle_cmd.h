/** @file obstacle_cmd.h
 *
 * @brief Command opcodes for obstacle_post_cmd(). Kernel-free so host tests
 *        and the private pack header can use them without tk/tkernel.h.
 */

#ifndef NAVIGATION_OBSTACLE_CMD_H
#define NAVIGATION_OBSTACLE_CMD_H

#define OBSTACLE_CMD_STOP  (5U)
#define OBSTACLE_CMD_RESET (6U)
#define OBSTACLE_CMD_START (7U)

#endif /* NAVIGATION_OBSTACLE_CMD_H */

/*** end of file ***/
