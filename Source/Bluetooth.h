#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#define BT_CMD_BIND     0x00
#define BT_CMD_PORT     0x01
#define BT_CMD_OPERATE  0x02
#define BT_CMD_LOOP     0x03

extern void BluetoothInit(byte task_id);
extern void Bluetooth_Handle(byte *msg);
#endif