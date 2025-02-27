/*
 * ble_commands.h
 *
 *  Created on: 26 gen 2021
 *      Author: UTPM9
 */

/***
 * Commands:
 * Bytes | Description
 * 0	 | HCI Packet Type (Indicates this is a command packet) (0x01 is command, 0x04 is event)
   1-2	 | Opcode
   3 	 | Parameter Length (n bytes follow this field)
 * */

/***
 * Events:
 * Bytes | Description
 * 0	 | HCI Packet Type (Indicates this is a command packet) (0x01 is command, 0x04 is event)
   1	 | Event Code (Command Complete is 0x0E)
   2	 | Parameter Length (n bytes follow this field)
   3	 | Number of HCI Command Packets Allowed
   4-5	 | Opcode of Command sent (Little Endian format)
   6     | Status
 * */

#ifndef INC_BLE_COMMANDS_H_
#define INC_BLE_COMMANDS_H_

#include <stdint.h>

#define READABLE 0x02
#define NOTIFIBLE 0x10
#define WRITABLE  0x04

uint8_t EVENT_STATUP_DATA[]={0x04,0xff,0x03,0x01,0x00,0x01};

uint8_t ACI_GATT_INIT[]={0x01,0x01,0xfd,0x00};
uint8_t ACI_GATT_INIT_COMPLETE[]={0x04,0x0e,0x04,0x01,0x01,0xfd,0x00};

uint8_t ACI_GAP_INIT[]={0x01,0x8a,0xfc,0x03,0x01,0x00,0x0d};
uint8_t ACI_GAP_INIT_COMPLETE[]={0x04,0x0e,0x0a,0x01,0x8a,0xfc,0x00};
uint8_t GAP_SERVICE_HANDLE[2];
uint8_t GAP_CHAR_NAME_HANDLE[2];
uint8_t GAP_CHAR_APP_HANDLE[2];

uint8_t ACI_GATT_UPDATE_CHAR_VALUE[]={0x01,0x06,0xfd,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
uint8_t ACI_GATT_UPDATE_CHAR_COMPLETE[]={0x04,0x0e,0x04,0x01,0x06,0xfd,0x00};

uint8_t ACI_GAP_SET_AUTH[]={0x01,0x86,0xfc,0x0c,0x00,0x00,0x01,0x00,0x07,0x10,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t ACI_GAP_SET_AUTH_RESP[]={0x04,0x0e,0x04,0x01,0x86,0xfc,0x00};

uint8_t ACI_HAL_SET_TX_POWER_LEVEL[]={0x01,0x0f,0xfc,0x02,0x01,0x04};
uint8_t ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE[]={0x04,0x0e,0x04,0x01,0x0f,0xfc,0x00};

uint8_t ACI_HAL_SET_STANDBY[]={0x01,0x13,0xfc,0x00};
uint8_t ACI_HAL_SET_STANDBY_COMPLETE[]={0x04,0x0e,0x04,0x01,0x13,0xfc,0x00};

uint8_t HCI_LE_SET_SCAN_RESPONSE_DATA[]={0x01,0x09,0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE[]={0x04,0x0e,0x04,0x01,0x09,0x20,0x00};

uint8_t ACI_GAP_SET_DISCOVERABLE[]={0x01,0x83,0xfc,0xff,0x00,0x40,0x06,0x40,0x06,0x01,0x00,0xff,0x09};
uint8_t ACI_GAP_SET_DISCOVERABLE_COMPLETE[]={0x04,0x0e,0x04,0x01,0x83,0xfc,0x00};

uint8_t ADD_PRIMARY_SERVICE[]={0x01,0x02,0xFD,0x13,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00,0x01,0x09};//3C 60 bytes il massimo di memoria per il servizio
uint8_t ADD_PRIMARY_SERVICE_COMPLETE[]={0x04,0x0e,0x06,0x01,0x02,0xFD,0x00};

uint8_t ADD_CUSTOM_CHAR[]={0x01,0x04,0xFD,0x1A,0xff,0xff,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00,0x10,0x00,0x02,0x00,0x01,0x10,0x01};
uint8_t ADD_CUSTOM_CHAR_COMPLETE[]={0x04,0x0e,0x06,0x01,0x04,0xFD,0x00};

uint8_t UPDATE_CHAR[]={0x01,0x06,0xFD,0x09,0xff,0xff,0xff,0xff,0x01,0x01,0x01,0x01,0x01,0x01,0x01};

uint8_t DISCONNECT[] = {0x01,0x06,0x04,0x00}; // TODO - fill this in
uint8_t EVENT_DISCONNECTED[]={0x04,0x05,0x03,0x00};

uint8_t EVENT_CONNECTED[] = {0x04, 0x3E, 0x13, 0x01, 0x00};

uint8_t ACI_GAP_SET_NON_DISCOVERABLE[] = {0x01,0x81,0xFC,0x00}; // TODO - fill this in
uint8_t ACI_GAP_SET_NON_DISCOVERABLE_COMPLETE[] = {0x04,0x0e,0x04,0x00, 0x01, 0x81, 0xFC,0x00}; // TODO - fill this in

#endif /* INC_BLE_COMMANDS_H_ */
