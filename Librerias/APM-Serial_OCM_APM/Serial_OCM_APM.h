
#ifndef ___Serial_OCM_APM___
#define ___Serial_OCM_APM___
#include <Arduino-comptibles.h>
#include <AP_Common.h>
#define APM

#ifdef OCM
    #include "libpandora_types.h"
    #include <Arduino-compatibles.h>
#endif

#define BAUDRATE 115200
#define ACK_WAIT     10

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

extern const AP_HAL::HAL& hal;

typedef unsigned char  byte;
typedef unsigned short word;
typedef enum msg_type {
    GET_AB            = 170,
    SET_AB            =  85,
    SHUTDOWN          = 219,
    E_SHUTDOWN        = 109,
    SETUP             = 182,
    ACKNOWLEDGE       =  36,
    GENERAL_DATA      = 254,
}msg_type;

typedef union byte_float{
        float f;
        char s[sizeof(float)];
}byte_float;

typedef union byte_word{
    byte b[sizeof(word)];
    word w;
}byte_word;

typedef struct Acknowledge{
		bool          state;
		unsigned long time;
		msg_type      type;
}Acknowledge;
                           
class Serial_OCM_APM{
    public:
        Serial_OCM_APM(void);
        ~Serial_OCM_APM(void);
    
        void send_message(msg_type, char*);
        void receive_message(void);
    
        byte Ack_checker(void);
                               
    private:
        //void uart_print(char);
    
        Acknowledge Ack_vec[3];
                               
};

#endif
