
#ifndef ___Serial_OCM_APM___
#define ___Serial_OCM_APM___

#define OCM

#ifdef OCM
    #include "libpandora_types.h"
    #include <Arduino-compatibles.h>
#endif

#define BAUDRATE 115200
#define ACK_WAIT     10

typedef enum msg_type {
    GET_AB            = 170,
    SET_AB            =  85,
    SHUTDOWN          = 219,
    E_SHUTDOWN        = 109,
    SETUP             = 182,
    ACKNOWLEDGE       =  36,
    GENERAL_DATA      = 254
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
