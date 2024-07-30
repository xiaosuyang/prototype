

#ifndef CSTRUCT_H
#define CSTRUCT_H

// #include <errno.h>
// #include <signal.h>
// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include<stdio.h>
// #include<stdlib.h>
// #include<unistd.h>
// #include<arpa/inet.h>
/****************************************************************************/
#include "ecrt.h"

/* Master 0, Slave 0
 * Vendor ID:       0x01222222
 * Product code:    0x00020310
 * Revision number: 0x00020211
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x7020, 0x01, 8}, /* Commond1 */
    {0x7020, 0x02, 8}, /* Commond2 */
    {0x7030, 0x01, 32}, /* Target Angle */
    {0x6000, 0x01, 32}, /* ssi */
    {0x6000, 0x02, 32}, /* ssi1 */
    {0x6010, 0x01, 16}, /* ain1 */
    {0x6010, 0x02, 16}, /* ain2 */
    {0x6010, 0x03, 16}, /* ain3 */
    {0x6010, 0x04, 16}, /* ain4 */
    {0x6010, 0x05, 16}, /* ain5 */
    {0x6010, 0x06, 16}, /* ain6 */
    {0x6010, 0x07, 16}, /* ain7 */
    {0x6010, 0x08, 16}, /* ain8 */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1601, 2, slave_0_pdo_entries + 0}, /* write commond of moulde */
    {0x1602, 1, slave_0_pdo_entries + 2}, /* write target of moulde */
    {0x1a00, 2, slave_0_pdo_entries + 3}, /* read ssi of moudle */
    {0x1a05, 8, slave_0_pdo_entries + 5}, /* read ain of moudle */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_0_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 1
 * Vendor ID:       0x00000b95
 * Product code:    0x00020130
 * Revision number: 0x00000002
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x7010, 0x01, 16}, /* current1 */
    {0x7010, 0x02, 16}, /* current2 */
    {0x7010, 0x03, 16}, /* current3 */
    {0x7010, 0x04, 16}, /* current4 */
    {0x7010, 0x05, 16}, /* current5 */
    {0x7010, 0x06, 16}, /* current6 */
    {0x0000, 0x00, 6}, /* Gap */
    {0x7011, 0x01, 16}, /* voltage1 */
    {0x7011, 0x02, 16}, /* voltage2 */
    {0x7011, 0x03, 16}, /* voltage3 */
    {0x7011, 0x04, 16}, /* voltage4 */
    {0x7011, 0x05, 16}, /* voltage5 */
    {0x7011, 0x06, 16}, /* voltage6 */
    {0x0000, 0x00, 6}, /* Gap */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1601, 7, slave_1_pdo_entries + 0}, /* Obj0x1601 */
    {0x1602, 7, slave_1_pdo_entries + 7}, /* Obj0x1602 */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_1_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 2
 * Vendor ID:       0x00000b95
 * Product code:    0x00020310
 * Revision number: 0x00020211
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x7020, 0x01, 8}, /* Commond1 */
    {0x7020, 0x02, 8}, /* Commond2 */
    {0x7030, 0x01, 32}, /* Target Angle */
    {0x6000, 0x01, 32}, /* ssi */
    {0x6000, 0x02, 32}, /* ssi1 */
    {0x6010, 0x01, 16}, /* ain1 */
    {0x6010, 0x02, 16}, /* ain2 */
    {0x6010, 0x03, 16}, /* ain3 */
    {0x6010, 0x04, 16}, /* ain4 */
    {0x6010, 0x05, 16}, /* ain5 */
    {0x6010, 0x06, 16}, /* ain6 */
    {0x6010, 0x07, 16}, /* ain7 */
    {0x6010, 0x08, 16}, /* ain8 */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1601, 2, slave_2_pdo_entries + 0}, /* write commond of moulde */
    {0x1602, 1, slave_2_pdo_entries + 2}, /* write target of moulde */
    {0x1a00, 2, slave_2_pdo_entries + 3}, /* read ssi of moudle */
    {0x1a05, 8, slave_2_pdo_entries + 5}, /* read ain of moudle */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_2_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_2_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 3
 * Vendor ID:       0x00000b95
 * Product code:    0x00020310
 * Revision number: 0x00020211
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x7020, 0x01, 8}, /* Commond1 */
    {0x7020, 0x02, 8}, /* Commond2 */
    {0x7030, 0x01, 32}, /* Target Angle */
    {0x6000, 0x01, 32}, /* ssi */
    {0x6000, 0x02, 32}, /* ssi1 */
    {0x6010, 0x01, 16}, /* ain1 */
    {0x6010, 0x02, 16}, /* ain2 */
    {0x6010, 0x03, 16}, /* ain3 */
    {0x6010, 0x04, 16}, /* ain4 */
    {0x6010, 0x05, 16}, /* ain5 */
    {0x6010, 0x06, 16}, /* ain6 */
    {0x6010, 0x07, 16}, /* ain7 */
    {0x6010, 0x08, 16}, /* ain8 */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1601, 2, slave_3_pdo_entries + 0}, /* write commond of moulde */
    {0x1602, 1, slave_3_pdo_entries + 2}, /* write target of moulde */
    {0x1a00, 2, slave_3_pdo_entries + 3}, /* read ssi of moudle */
    {0x1a05, 8, slave_3_pdo_entries + 5}, /* read ain of moudle */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_3_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_3_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 4
 * Vendor ID:       0x00000b95
 * Product code:    0x00020310
 * Revision number: 0x00020211
 */

ec_pdo_entry_info_t slave_4_pdo_entries[] = {
    {0x7020, 0x01, 8}, /* Commond1 */
    {0x7020, 0x02, 8}, /* Commond2 */
    {0x7030, 0x01, 32}, /* Target Angle */
    {0x6000, 0x01, 32}, /* ssi */
    {0x6000, 0x02, 32}, /* ssi1 */
    {0x6010, 0x01, 16}, /* ain1 */
    {0x6010, 0x02, 16}, /* ain2 */
    {0x6010, 0x03, 16}, /* ain3 */
    {0x6010, 0x04, 16}, /* ain4 */
    {0x6010, 0x05, 16}, /* ain5 */
    {0x6010, 0x06, 16}, /* ain6 */
    {0x6010, 0x07, 16}, /* ain7 */
    {0x6010, 0x08, 16}, /* ain8 */
};

ec_pdo_info_t slave_4_pdos[] = {
    {0x1601, 2, slave_4_pdo_entries + 0}, /* write commond of moulde */
    {0x1602, 1, slave_4_pdo_entries + 2}, /* write target of moulde */
    {0x1a00, 2, slave_4_pdo_entries + 3}, /* read ssi of moudle */
    {0x1a05, 8, slave_4_pdo_entries + 5}, /* read ain of moudle */
};

ec_sync_info_t slave_4_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_4_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_4_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 5
 * Vendor ID:       0x00000b95
 * Product code:    0x00020310
 * Revision number: 0x00020211
 */

ec_pdo_entry_info_t slave_5_pdo_entries[] = {
    {0x7020, 0x01, 8}, /* Commond1 */
    {0x7020, 0x02, 8}, /* Commond2 */
    {0x7030, 0x01, 32}, /* Target Angle */
    {0x6000, 0x01, 32}, /* ssi */
    {0x6000, 0x02, 32}, /* ssi1 */
    {0x6010, 0x01, 16}, /* ain1 */
    {0x6010, 0x02, 16}, /* ain2 */
    {0x6010, 0x03, 16}, /* ain3 */
    {0x6010, 0x04, 16}, /* ain4 */
    {0x6010, 0x05, 16}, /* ain5 */
    {0x6010, 0x06, 16}, /* ain6 */
    {0x6010, 0x07, 16}, /* ain7 */
    {0x6010, 0x08, 16}, /* ain8 */
};

ec_pdo_info_t slave_5_pdos[] = {
    {0x1601, 2, slave_5_pdo_entries + 0}, /* write commond of moulde */
    {0x1602, 1, slave_5_pdo_entries + 2}, /* write target of moulde */
    {0x1a00, 2, slave_5_pdo_entries + 3}, /* read ssi of moudle */
    {0x1a05, 8, slave_5_pdo_entries + 5}, /* read ain of moudle */
};

ec_sync_info_t slave_5_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_5_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_5_pdos + 2, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 6
 * Vendor ID:       0x00000b95
 * Product code:    0x00020310
 * Revision number: 0x00020211
 */

ec_pdo_entry_info_t slave_6_pdo_entries[] = {
    {0x7020, 0x01, 8}, /* Commond1 */
    {0x7020, 0x02, 8}, /* Commond2 */
    {0x7030, 0x01, 32}, /* Target Angle */
    {0x6000, 0x01, 32}, /* ssi */
    {0x6000, 0x02, 32}, /* ssi1 */
    {0x6010, 0x01, 16}, /* ain1 */
    {0x6010, 0x02, 16}, /* ain2 */
    {0x6010, 0x03, 16}, /* ain3 */
    {0x6010, 0x04, 16}, /* ain4 */
    {0x6010, 0x05, 16}, /* ain5 */
    {0x6010, 0x06, 16}, /* ain6 */
    {0x6010, 0x07, 16}, /* ain7 */
    {0x6010, 0x08, 16}, /* ain8 */
};

ec_pdo_info_t slave_6_pdos[] = {
    {0x1601, 2, slave_6_pdo_entries + 0}, /* write commond of moulde */
    {0x1602, 1, slave_6_pdo_entries + 2}, /* write target of moulde */
    {0x1a00, 2, slave_6_pdo_entries + 3}, /* read ssi of moudle */
    {0x1a05, 8, slave_6_pdo_entries + 5}, /* read ain of moudle */
};

ec_sync_info_t slave_6_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, slave_6_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, slave_6_pdos + 2, EC_WD_DISABLE},
    {0xff}
};


#endif
