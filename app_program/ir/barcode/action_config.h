/** @file action_config.h
 * @brief Editable letter bindings for barcode commands.
 */

#ifndef BARCODE_ACTION_CONFIG_H
#define BARCODE_ACTION_CONFIG_H

/* Assign a distinct uppercase A-Z letter to each binding. */
#ifndef BARCODE_CHAR_STOP
#define BARCODE_CHAR_STOP ('H')
#endif
#ifndef BARCODE_CHAR_ALT_STOP
#define BARCODE_CHAR_ALT_STOP ('Z')
#endif
#ifndef BARCODE_CHAR_LEFT
#define BARCODE_CHAR_LEFT ('L')
#endif
#ifndef BARCODE_CHAR_RIGHT
#define BARCODE_CHAR_RIGHT ('R')
#endif
#ifndef BARCODE_CHAR_STRAIGHT
#define BARCODE_CHAR_STRAIGHT ('S')
#endif
#ifndef BARCODE_CHAR_UTURN
#define BARCODE_CHAR_UTURN ('U')
#endif

#endif /* BARCODE_ACTION_CONFIG_H */

/*** end of file ***/
