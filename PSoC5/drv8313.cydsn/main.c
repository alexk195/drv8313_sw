/*******************************************************************************
* Rotate BLDC motor using PSoC5 and TI8313
********************************************************************************
*
* Summary:
*  On power on green led should go on. Then motor will start movement.
*  If red led goes on then N_FAULT signal indicates an error.
*  Motor should accelerate up to 1 revolution per second. Rotate for 10 seconds
*  with constant speed then decelerates to full stop. Green LED goes off then.
*  
*  www.el08.de/ti8313
*  github.com/alexk195/drv8313_sw
*******************************************************************************/

#include <device.h>

#include <stdio.h>
#include <math.h>


// LUT size for sine values of phases
#define PHASES_SIZE (3000)
#define PHASES_OFFS_0 (0)
#define PHASES_OFFS_1 (PHASES_SIZE/3)
#define PHASES_OFFS_2 (PHASES_SIZE*2/3)

// LUT storage for PWM values
uint8_t phases[PHASES_SIZE];

// current PWM values
uint8_t pwm[3];

// property of the motor: number of poles
#define numberOfPoles 6

// actual degree of rotor for one phase cycle
#define degreePerPhase (360.0/(float)numberOfPoles)

// Interrupt frequency 10kHz
#define ISR_FREQ 10000

#define degreeToPhase ((float)(PHASES_SIZE)/(float)(degreePerPhase))

// phase 0..PHASES_SIZE
uint16_t phasei = 0;
float phasef = 0.0;

// maximum power 0..1. 
const float maxpwr = 0.3;

// current speed in phase units
float speed = 0.0;

// maximum speed in degree per second
#define maxSpeedDegreePerSecond 360.0

// maximum speed in phase units
const float maxspeed = maxSpeedDegreePerSecond*degreeToPhase/(float)ISR_FREQ;


// init LUT table according to given power
void initPhases(uint8_t * p, uint16_t size, float power)
{
    for (uint16_t i=0;i<size;i++)
    {
        float phase = (float)i*(2.0f*(float)M_PI/(float)size);   
        float t = (sin(phase)*power+1.0)*127.0;
        if (t>=255.0) t=255.0;
        if (t<0.0) t=0.0;
        p[i] = (uint8_t)t;
    }
}

// set pwm values according to phase
void phaseToPWM(uint16_t _phase, uint8_t * _pwm)
{
   _pwm[0] = phases[(_phase+PHASES_OFFS_0)%PHASES_SIZE];
   _pwm[1] = phases[(_phase+PHASES_OFFS_1)%PHASES_SIZE];
   _pwm[2] = phases[(_phase+PHASES_OFFS_2)%PHASES_SIZE];
}

#define CTR_EN1 1
#define CTR_EN2 2
#define CTR_EN3 4
#define CTR_N_RESET 8
#define CTR_N_SLEEP 16
#define CTR_PWM_RESET 32

#define ST_N_COMPO 1
#define ST_N_FAULT 2


void bldc_isr_handler()
{
    //LOOP_OUT_Write(1);
    
    // update phase
    phasef += speed;
    
    // clamp to valid values
    while (phasef >= PHASES_SIZE)
    {
       phasef -= PHASES_SIZE;
    }
    while (phasef < 0)
    {
        phasef += PHASES_SIZE;
    }
 
    // get pwm values for all bridges
    phaseToPWM((uint16_t)phasef,pwm);

    // set the pwm values
    PWM_1_WriteCompare(pwm[0]);
    PWM_2_WriteCompare(pwm[1]);
    PWM_3_WriteCompare(pwm[2]);
     
    // check for errors
    if (0 == (Status_Reg_1_Read()  & ST_N_FAULT))
    {
        LED_RED_Write(1); 
    }
    //LOOP_OUT_Write(0);    
}

CY_ISR_PROTO(bldc_interrupt1);

CY_ISR(bldc_interrupt1)
{
   bldc_isr_handler();
}

int main()
{
    /* Prepare components */
    Control_Reg_1_Write(0);
    CyDelay(1000);
    
    Control_Reg_1_Write(CTR_N_RESET+CTR_N_SLEEP); // release 'reset' and 'sleep'
    uint8_t s = Status_Reg_1_Read();

    // start pwm components
    PWM_1_Start();    
    PWM_2_Start(); 
    PWM_3_Start(); 
    
    Control_Reg_1_Write(CTR_N_RESET+CTR_N_SLEEP + CTR_EN1+CTR_EN2+CTR_EN3); // enable all

    s = Status_Reg_1_Read();
    
    if (0 == (s & ST_N_FAULT))
    {
        // Error detected: do not startup, indicate by LED
        LED_RED_Write(1);    
        LED_GREEN_Write(0);
    }
    else
    {
        // No errors: starting up
        LED_RED_Write(0);    
        LED_GREEN_Write(1);
        
        initPhases(phases,sizeof(phases)/sizeof(phases[0]),maxpwr);

        {
            // register interrupt           
            isr_1_StartEx(bldc_interrupt1);
        
            // enable all interrupts
            CyGlobalIntEnable;
        
            // sleep for 200 ms
            CyDelay(200);
             
            // set initial speed
            speed = 0.01;
           
            // accelerate
            for (;speed < maxspeed;speed *= 1.1f)
            {
                CyDelay(100);
            }
            
            // rotate with max speed for 10 seconds
            speed = maxspeed;

            CyDelay(10000);
            
            // decelerate
            for (;speed > 0.01; speed *= 0.9f)
            {
                CyDelay(100);
            }
        
            // set speed to 0 -> stop
            speed = 0.0;
            
            LED_GREEN_Write(0);
        
            // disable all bridges again
            Control_Reg_1_Write(0); 
        }

        for(;;)
        {
        }

        // optionally also stop interrupts
        //isr_1_Stop();
    }
    
      
}



/* [] END OF FILE */

