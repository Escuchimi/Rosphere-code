#ifndef _Serial_APM_RPI_h
#define _Serial_APM_RPI_h

#include <Serial_OCM_APM.h>

#define BAUDRATE 115200

enum msg_type_RPI {
    MOVEMENT_DATA     =  36,
    IMU               =  85,
    ALARMS            = 109,
    GPS_DATA          = 170
};

class Serial_APM_RPI{
    public:
    Serial_APM_RPI(void);
    ~Serial_APM_RPI(void);
    
    void send_message(msg_type_RPI, char*);
    void receive_message(void);
};
#endif
