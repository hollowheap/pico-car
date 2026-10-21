/** @file encoder_input_selftest.h
 * @brief Isolated input-override and software-forced interrupt diagnostic.
 */
#ifndef ENCODER_INPUT_SELFTEST_H
#define ENCODER_INPUT_SELFTEST_H

#include <stdbool.h>

/* Core 0 task only, after capture init. Restores pin controls, enables and
 * the real-input counter baseline before returning. Does not drive pins.
 * Success does not validate electrical inputs or physical edge detection. */
bool encoder_input_selftest (void);

#endif /* ENCODER_INPUT_SELFTEST_H */

/*** end of file ***/
