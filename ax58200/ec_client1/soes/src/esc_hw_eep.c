/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

 /** \file
 * \brief
 * ESC hardware specific EEPROM emulation functions.
 */

#include "cc.h"
#include "esc.h"
//#include "esc_hw_eep.h"

#include <string.h>

/** Initialize EEPROM emulation (load default data, validate checksums, ...).
 *
 */
void EEP_init (void)
{
}

/** EEPROM emulation controller side periodic task.
 *
 */
void EEP_hw_process (void)
{
}

/** EEPROM read function
 *
 * @param[in]   addr     = EEPROM byte address
 * @param[out]  data     = pointer to buffer of output data
 * @param[in]   count    = number of bytes to read
 * @return 0 on OK, 1 on error
 */
int8_t EEP_read (uint32_t addr, uint8_t *data, uint16_t count)
{
   return 0;
}

/** EEPROM write function
 *
 * @param[in]   addr     = EEPROM byte address
 * @param[out]  data     = pointer to buffer of input data
 * @param[in]   count    = number of bytes to write
 * @return 0 on OK, 1 on error
 */
int8_t EEP_write (uint32_t addr, uint8_t *data, uint16_t count)
{
   return 0;
}

