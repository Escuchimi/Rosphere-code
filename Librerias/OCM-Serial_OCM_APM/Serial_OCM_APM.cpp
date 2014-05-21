#include "Serial_OCM_APM.h"


Serial_OCM_APM::Serial_OCM_APM(void)
{
    for(int i = 0; i>3; i++){
        Ack_vec[i].state         = false;
        Ack_vec[i].time          = 0;
        Ack_vec[i].type      = ACKNOWLEDGE;
}
    
#ifdef APM
    hal.uartC->begin(BAUDRATE);
#elif defined(OCM)
    Serial3.begin(BAUDRATE);
#endif
}



Serial_OCM_APM::~Serial_OCM_APM()
{
    for(int i = 0; i>3; i++){
        Ack_vec[i].state         = false;
        Ack_vec[i].time          = 0;
        Ack_vec[i].type      = ACKNOWLEDGE;
    }
}

void Serial_OCM_APM::send_message(msg_type type, char *value)
{
    
    char *message;
    char checksum = (char)0;
    
    switch(type){
            
        case ACKNOWLEDGE:
            message = (char*) malloc (5);
        
            checksum += (message[0] = (char)255);
            checksum += (message[1] = (char)type);
            checksum += (message[2] = (char)value[0]);
            message[3]              = checksum;
            message[4]              = '\0';
            
            break;
            
        case SET_AB:
            message = (char*) malloc (8);
        
            checksum += (message[0] = (char)255);
            checksum += (message[1] = (char)type);
            checksum += (message[2] = (char)value[0]);
            checksum += (message[3] = (char)value[1]);
            checksum += (message[4] = (char)value[2]);
            checksum += (message[5] = (char)value[3]);
            message[6]              = checksum;
            message[7]              = '\0';
            
            break;
                     
        case GET_AB:
            message = (char*) malloc (10);
        
            checksum += (message[0] = (char)255);
            checksum += (message[1] = (char)type);
            checksum += (message[2] = (char)value[0]);
            checksum += (message[3] = (char)value[1]);
            checksum += (message[4] = (char)value[2]);
            checksum += (message[5] = (char)value[3]);
            checksum += (message[6] = (char)value[4]);
            checksum += (message[7] = (char)value[5]);
            message[8]              = checksum;
            message[9]              = '\0';
                     
            break;
            
        case GENERAL_DATA:
            message = (char*) malloc (16);
            
            checksum += (message[0]  = (char)255);
            checksum += (message[1]  = (char)type);
            checksum += (message[2]  = (char)value[0]);
            checksum += (message[3]  = (char)value[1]);
            checksum += (message[4]  = (char)value[2]);
            checksum += (message[5]  = (char)value[3]);
            checksum += (message[6]  = (char)value[4]);
            checksum += (message[7]  = (char)value[5]);
            checksum += (message[8]  = (char)value[6]);
            checksum += (message[9]  = (char)value[7]);
            checksum += (message[10] = (char)value[8]);
            checksum += (message[11] = (char)value[9]);
            checksum += (message[12] = (char)value[10]);
            checksum += (message[13] = (char)value[11]);
            message[14]              = checksum;
            message[15]              = '\0';
            
            break;
            
        default:
                     
            message = (char*) malloc (4);
                     
            checksum += (message[0] = (char)255);
            checksum += (message[1] = (char)type);
            message[2]              = checksum;
            message[3]              = '\0';
            
            break;
    }

#ifdef APM
                     
                     hal.uartC->printf(message);
    
#elif defined(OCM)
                     
                     Serial3.print(message);
                     
#endif
    
    free (message);
    
    for(int i = 0; i<3; i++){
        if(Ack_vec[i].state == false){
            Ack_vec[i].state = true;
            Ack_vec[i].time  = millis();
            Ack_vec[i].type  = type;
            break;
        }
    }
}

                     
void Serial_OCM_APM::receive_message(void){
    int type = 0;
    byte     checksum;
    
#ifdef APM
    if(hal.uartC->available() >= 2)
    {
        if(hal.uartC->read() != 255) return;
        type = hal.uartC->read();
    }
#elif defined(OCM)
    if(Serial3.available() >= 2 )
    {
        if(Serial3.read() != 255) return;
        type = Serial3.read();
    }
#endif
    
    if(type != 0)
    {
        delayMicroseconds(700);
        switch(type){
                
#ifdef OCM
            case SET_AB:
            {
                extern word steer;
                extern word drive;
                
                byte_word drive_read;
                byte_word steer_read;

                drive_read.b[0] = Serial3.read();
                drive_read.b[1] = Serial3.read();
                
                steer_read.b[0] = Serial3.read();
                steer_read.b[1] = Serial3.read();
                
                checksum = 255 + type + drive_read.b[0] + drive_read.b[1] + steer_read.b[0] + steer_read.b[1];
                if(checksum != Serial3.read())return;

                drive    = drive_read.w;
                steer    = steer_read.w;
            }break;
#endif
         
#ifdef APM
            case GET_AB:
            {
                extern float velocity_real;
                extern word  angle;
                
                byte       lowbyte_angle;
                byte       highbyte_angle;
                byte_float vel;
                
                vel.s[0] = hal.uartC->read();
                vel.s[1] = hal.uartC->read();
                vel.s[2] = hal.uartC->read();
                vel.s[3] = hal.uartC->read();
                
                angle_read.b[0] = hal.uartC->read();
                angle_read.b[1] = hal.uartC->read();
                
                checksum = 255 + type + vel.s[0] + vel.s[1] + vel.s[2] + vel.s[3];
                if(checksum != hal.uartC->read())return;
                
                angle         = angle_read.w;
                velocity_real = vel.f;


            }break;
#endif
 
#ifdef OCM
            case ACKNOWLEDGE:
            {
                byte ack;

                ack = Serial3.read();
                
                checksum = 255 + type + ack;
                if(checksum != Serial3.read())return;
                
                for(int i = 0; i>3; i++)
                    if(Ack_vec[i].state == true && ack == Ack_vec[i].type)Ack_vec[i].state = false;
            }break;
#endif
 
#ifdef APM //---------------------------------------------------------------------------
            case GENERAL_DATA:
            {
                extern float temperature_drive;
                extern float temperature_steer;
                extern float voltage;
                
                byte_float drive_temp;
                byte_float steer_temp;
                byte_float       volt;


                drive_temp.s[0] = hal.uartC->read();
                drive_temp.s[1] = hal.uartC->read();
                drive_temp.s[2] = hal.uartC->read();
                drive_temp.s[3] = hal.uartC->read();
                
                steer_temp.s[0] = hal.uartC->read();
                steer_temp.s[1] = hal.uartC->read();
                steer_temp.s[2] = hal.uartC->read();
                steer_temp.s[3] = hal.uartC->read();
                
                volt.s[0]       = hal.uartC->read();
                volt.s[1]       = hal.uartC->read();
                volt.s[2]       = hal.uartC->read();
                volt.s[3]       = hal.uartC->read();
                
                checksum = 255 + type + drive_temp.s[0] + drive_temp.s[1] + drive_temp.s[2] + drive_temp.s[3] + steer_temp.s[0] + steer_temp.s[0] + steer_temp.s[0] + steer_temp.s[0] + volt.s[0] + volt.s[1] + volt.s[2] + volt.s[3];
                
                if(checksum != hal.uartC->read())return;

                temperature_drive = drive_temp.f;
                temperature_steer = steer_temp.f;
                voltage           = volt.f;
            }break;
#endif
                
#ifdef OCM
            case E_SHUTDOWN:
            {
                checksum = 255+type;
                if(checksum !=Serial3.read())return;
                
                extern bool emergency_shutdown;
                
                emergency_shutdown = true;
            }
#endif
                
        
            default:
                //HACE FALTA RELLENAR CON EL RESTO DE CASOS
                break;
        }
    }
    
}
                         
// REVISAR los .type
byte Serial_OCM_APM::Ack_checker(void)
{
    byte b = 0x00;
    for(int i = 0; i<3; i++){
        if(Ack_vec[i].type == ACKNOWLEDGE || Ack_vec[i].type == GET_AB) Ack_vec[i].state = false;
        
        if(Ack_vec[i].state == true){
            if(Ack_vec[i].time >(millis()- ACK_WAIT)){
                switch (Ack_vec[i].type){
                    case 85:
                        b = b | 0x80;
                        break;
                    case 0xD0:
                        b = b | 0x20;
                        break;
                    case 0xB7:
                        b = b | 0x08;
                        break;
                    case 0xB0:
                        b = b | 0x02;
                        break;
                    default:
                        break;
                }
            }
            return b;
        }
        
    }
}



