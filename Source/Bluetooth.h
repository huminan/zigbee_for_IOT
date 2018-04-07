#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#define BT_CMD_BIND     0x00
#define BT_CMD_PORT     PORT_INIT_CLUSTER
#define BT_CMD_OPERATE  OPERATE_CLUSTER
#define BT_CMD_LOOP     LOOP_OPERATE_CLUSTER

extern void BluetoothInit(byte task_id);
extern void Bluetooth_Handle(byte *msg);
#endif