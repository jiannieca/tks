#ifndef _APP_BMU_H
#define _APP_BMU_H
#include "data.h"
#include "applicfg.h"
#include "bmu.h"
/***************************************************************************/
/* SDO (un)packing macros */

/** Returns the command specifier (cs, ccs, scs) from the first byte of the SDO
*/
#define getSDOcs(byte) (byte >> 5)

/** Returns the number of bytes without data from the first byte of the SDO. Coded in 2 bits
*/
#define getSDOn2(byte) ((byte >> 2) & 3)

/** Returns the number of bytes without data from the first byte of the SDO. Coded in 3 bits
*/
#define getSDOn3(byte) ((byte >> 1) & 7)

/** Returns the transfer type from the first byte of the SDO
*/
#define getSDOe(byte) ((byte >> 1) & 1)

/** Returns the size indicator from the first byte of the SDO
*/
#define getSDOs(byte) (byte & 1)

/** Returns the indicator of end transmission from the first byte of the SDO
*/
#define getSDOc(byte) (byte & 1)

/** Returns the toggle from the first byte of the SDO
*/
#define getSDOt(byte) ((byte >> 4) & 1)

/** Returns the index from the bytes 1 and 2 of the SDO
*/
#define getSDOindex(byte1, byte2) (((UNS16)byte2 << 8) | ((UNS16)byte1))

/** Returns the subIndex from the byte 3 of the SDO
*/
#define getSDOsubIndex(byte3) (byte3)

/** Returns the subcommand in SDO block transfer
*/
#define getSDOblockSC(byte) (byte & 3)



parseBroadMsg(Message *d, UNS8 nodeId);
#endif

