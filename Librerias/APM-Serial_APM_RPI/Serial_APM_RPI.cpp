#include <Serial_APM_RPI.h>

Serial_APM_RPI::Serial_APM_RPI(void)
{
}
Serial_APM_RPI::~Serial_APM_RPI()
{
}

void Serial_APM_RPI::send_message(msg_type_RPI type, char *value)
{
    char *message;
    char checksum = (char)0;
    
    switch(type){
            
        case MOVEMENT_DATA:
            message = (char*) malloc(12);
            
            checksum += (message[0] = (char)254);
            checksum += (message[1] = (char)type);
            checksum += (message[2] = (char)value[0]);
            checksum += (message[3] = (char)value[1]);
            checksum += (message[4] = (char)value[2]);
            checksum += (message[5] = (char)value[3]);
            checksum += (message[6] = (char)value[4]);
            checksum += (message[7] = (char)value[5]);
            checksum += (message[8] = (char)value[6]);
            checksum += (message[9] = (char)value[7]);
            message[10]             = checksum;
            message[11]             = '\0';
            
            break;
        
        case IMU:
            message = (char*) malloc (16);
            
            checksum += (message[0]  = (char)254);
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
            
        case GPS_DATA:
            break;
        
        case ALARMS:
            // ESTO  HAY QUE RELLENARLO
            break;
            
        default:
            break;
    }
    hal.console->printf(message);
    
    free(message);
    
}


extern float velocity_desired;
extern float heading_desired;

void Serial_APM_RPI::receive_message(void)
{
    int type = 0;
    byte     checksum;
    
    //PROVISIONAL--------------------------------------------------------------------------------------------------
    //Cuando haya mas tipos de mensaje habra que completarlo
    
    if(10 <= hal.console->available())
    {
        byte_float raw_drive;
        byte_float raw_steer;
        
        char buffer[10];
        
        for(int i = 0; i < 10; i++) buffer[i] = hal.console->read();
        
        if(hal.uartC->read() != 254) return;
        else if(buffer[9] != char(255))return;
        
        else
        {
            for(int i = 0; i < 4; i++)
            {
                raw_drive.s[i] = buffer[i+1];
                raw_steer.s[i] = buffer[i+5];
            }
            
            velocity_desired = raw_drive.f;
            heading_desired  = raw_steer.f;

        }
    }
    //PROVISIONAL---------------------------------------------------------------------------------------------------
}

