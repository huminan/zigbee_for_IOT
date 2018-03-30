#ifndef DEVICE_H
#define DEVICE_H
// These constants are only for example and should be changed to the
// device's needs

// EndPoint
#define SYS_ENDPOINT           2
#define ZB_ENDPOINT            3
#define BUTTON_ENDPOINT        10
#define MOTOR_ENDPOINT         110
#define LED_ENDPOINT           60
#define OBSERVE_ENDPOINT       130  

// Cluster ID MAX
#define SYS_MAX_CLUSTERS       1
#define ZB_MAX_CLUSTERS        1

#define BUTTON_MAX_CLUSTERS    3    
#define MOTOR_MAX_CLUSTERS     3    // stop, forward, backward
#define LED_MAX_CLUSTERS       3    // toggle light, toggle dim, flash
#define OBSERVE_MAX_CLUSTERS   1

// Cluster ID
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
#define LED_TYPE_ID            0x04
#define OBSERVE_TYPE_ID        0x08
  //...   0x04 0x08 0x10 ...

#endif