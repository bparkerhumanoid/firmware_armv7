 /** \file
 * \brief
 * ESC hardware layer functions.
 *
 * Function to read and write commands to the ESC. Used to read/write ESC
 * registers and memory.
 */

#include <stdint.h>

#include "esc.h"
#include "esc_hw.h"


/** ESC read function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to read
 * @param[out]  buf         = pointer to buffer to read in
 * @param[in]   len         = number of bytes to read
 */
void ESC_read (uint16_t address, void *buf, uint16_t len)
{
}

/** ESC write function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to write
 * @param[out]  buf         = pointer to buffer to write from
 * @param[in]   len         = number of bytes to write
 */
void ESC_write (uint16_t address, void *buf, uint16_t len)
{
}

/** ESC emulated EEPROM handler
 */
void ESC_eep_handler(void)
{
//   EEP_process ();
//   EEP_hw_process();
}

void ESC_reset (void)
{
}

void ESC_init (const esc_cfg_t * cfg)
{
}

