
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

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
#include <Serial_OCM_APM.h>
#include <Serial_APM_RPI.h>

// INS and Baro declaration
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_InertialSensor_MPU6000 ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins( &adc );
AP_Baro_BMP085 baro;
#else
AP_InertialSensor_HIL ins;
#endif

AP_Compass_HMC5843 compass;

GPS *g_gps;

AP_GPS_Auto g_gps_driver(&g_gps);

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(ins, baro, g_gps);

AP_Baro_HIL barometer;


#define HIGH 1
#define LOW 0

#define SEND_MOVEMENT_RATE   500
#define SEND_IMU_RATE        500


Serial_OCM_APM communication_OCM;
Serial_APM_RPI communication_RPI;

void update_motor_controler(void);


//Real values of variables, measured in the motor controller or with the IMU 
float          velocity_real;
float          roll_real;
float          heading_real;

//Desired values, setpoints of the regulators, modified by the model and/or (depends if it's teleoperated) directly from the Raspberry Pi commands
float          velocity_desired;
float          roll_desired;
float          heading_desired;
float          drive_torque;


unsigned long  time;   

// Internal variables needed in order to make the program work
unsigned long movement_entrance_time;
float         last_velocity_desired;
float         last_roll_desired;

void setup(void)
{
  velocity_real          = 0.0;
  roll_real              = 0.0;
  heading_real          = 0.0;
  
  velocity_desired       = 0.0;
  roll_desired           = 0.0;
  heading_desired        = 0.0;
  
  movement_entrance_time = hal.scheduler->millis();
  
  hal.console->begin(57600);
}

void loop(void)
{
  time = hal.scheduler->millis();
  
  if(time - movement_entrance_time >= SEND_MOVEMENT_RATE)
  {
    char       content[8];
    
    byte_float vel;
    byte_float head;
    
    vel.f  = velocity_real;
    head.f = (compass.use_for_yaw() ? ToDeg(heading_real) : 0.0);
    
    for(int i=0; i<4; i++)
    {
      content[i]   = vel.s[i];
      content[i+4] = head.s[i];
    }
    
    communication_RPI.send_message(MOVEMENT_DATA,content);  
  }
  
  update_motor_controller();
}

AP_HAL_MAIN();


void update_motor_controller()
{
      if(last_velocity_desired != velocity_desired || last_roll_desired != roll_desired)
      {
        char content[4];
        
        byte_word drive;
        byte_word steer;
    
        if      (drive_torque <  0.0) drive.w = mapfloat(drive_torque, 0.0, -5.0, 0, 1023);
        else if (drive_torque == 0.0) drive.w = 1024;
        else if (drive_torque >  0.0) drive.w = mapfloat(drive_torque, 0.0, 5.0, 1025, 2047);
      
        steer.w = mapfloat(roll_desired, -40.0, 40.0, 40, 980);

      
        content[0] = drive.b[0];
        content[1] = drive.b[1];
        content[2] = steer.b[0];
        content[3] = steer.b[1];
      
      communication_OCM.send_message(SET_AB,content);
      }
  
}

int mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}



