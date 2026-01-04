#pragma once

/*
 * Robust Binary I/O v2 - Command Definitions
 *
 * Consolidated command structure using 'C' request / 'c' response packet types.
 * See docs/RobustBinaryIOv2.md for full protocol specification.
 */

/* Command IDs */
#define CMD_WRITE       0x01  /* Save settings to flash */
#define CMD_CALIBRATE   0x02  /* Run sensor calibration */
#define CMD_BOOTLOADER  0x03  /* Enter bootloader mode */

/* Command response status codes */
#define CMD_STATUS_OK      0x00  /* Success */
#define CMD_STATUS_ERROR   0x01  /* Error */
#define CMD_STATUS_BUSY    0x02  /* Busy (e.g., calibration in progress) */
#define CMD_STATUS_UNKNOWN 0xFF  /* Unknown command */

/* Packet types */
#define PKT_CMD_REQUEST   'C'  /* 0x43 - Command request */
#define PKT_CMD_RESPONSE  'c'  /* 0x63 - Command response */
