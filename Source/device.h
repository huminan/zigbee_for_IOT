#ifndef DEVICE_H
#define DEVICE_H
// These constants are only for example and should be changed to the
// device's needs

// EndPoint
#define SYS_ENDPOINT           2
#define ZB_ENDPOINT            3
#define BUTTON_ENDPOINT        10
#define MOTOR_ENDPOINT         110
#define SWITCH_ENDPOINT        60
#define OBSERVE_ENDPOINT       130  

// Cluster ID MAX
#define SYS_MAX_CLUSTERS       1
#define ZB_MAX_CLUSTERS        1

#define BUTTON_MAX_CLUSTERS    4    // PORT_INIT_CLUSTER, OPERATE_CLUSTER, LOOP_OPERATE_CLUSTER, DELETE_CLUSTER
#define MOTOR_MAX_CLUSTERS     4    // PORT_INIT_CLUSTER, OPERATE_CLUSTER, LOOP_OPERATE_CLUSTER, DELETE_CLUSTER
#define SWITCH_MAX_CLUSTERS    4    // PORT_INIT_CLUSTER, OPERATE_CLUSTER, LOOP_OPERATE_CLUSTER, DELETE_CLUSTER
#define OBSERVE_MAX_CLUSTERS   1

// Cluster ID
#define PORT_INIT_CLUSTER      0x1000
#define OPERATE_CLUSTER        0x1001
#define LOOP_OPERATE_CLUSTER   0x1002
#define DELETE_CLUSTER         0x1003

#define SYS_CLUSTERID          0x0001
#define ZB_CLUSTERID           0x0002

#define BUTTON_OPEN            0x0001
#define BUTTON_CLOSE           0x0002
#define BUTTON_TRIGGER         0x0003

#define MOTOR_FORWARD          0x0001
#define MOTOR_BACKWARD         0x0002
#define MOTOR_STOP             0x0003

#define LED_LIGHT              0x0001
#define LED_DIM                0x0002
#define LED_FLASH              0x0003

#define OBSERVE_DATA           0x0001

// Terminator Type ID   -------  seems no use now
#define BUTTON_TYPE_ID         0x01
#define MOTOR_TYPE_ID          0x02
#define SWITCH_TYPE_ID            0x04
#define OBSERVE_TYPE_ID        0x08
  //...   0x04 0x08 0x10 ...

// PORT NUMBERS
#define P2_SWITCH_NUM       4


#define OPERATE_MSG_NUM     6
typedef struct
{
    uint8 value;
    uint16 level;
    uint8 hour;
    uint8 min;
    uint8 sec;
}sensor_msg_t;

typedef struct {
  sensor_msg_t *msg;
  uint8 total;      /* how much commands */
  uint8 status;     /* what is doing now */
  uint8 port;       /* use which port */
  uint16 command;   /* command : Loop or not*/
} SensorControl_t;


typedef struct
{
  osal_event_hdr_t  hdr;
  uint8             *msg;
} OSALSerialData_t;

#endif