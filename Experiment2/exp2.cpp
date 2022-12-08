/*
* main.c
*
* Created: 9/2/2022 2:06:25 AM
*  Author: CCLab
*/


// -O0
// 7372800Hz


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"


#define F_CPU = 7372800
unsigned int mode = 0;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char left_motor;
unsigned char right_motor;
unsigned char l = 0;
unsigned char c = 0;
unsigned char r = 0;
unsigned char PortBRestore = 0;
unsigned int pwm = 0;
unsigned int P , I, D = 0;
unsigned int Kp = 1.8;
unsigned int Kd, Ki=0 ; 


unsigned int pe = 0;
unsigned int motorspeed= 0;
int e=0;
unsigned int basespeed = 55;
unsigned int motorspeeda;
unsigned int motorspeedb;


void motion_pin_config (void)
{
DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
unsigned char PortBRestore = 0;


Direction &= 0x0F;                         // removing upper nibble as it is not needed
PortBRestore = PORTB;                         // reading the PORTB's original status
PortBRestore &= 0xF0;                         // setting lower direction nibble to 0
PortBRestore |= Direction;         // adding lower nibble for direction command and restoring the PORTB status
PORTB = PortBRestore;                         // setting the command to the port
}


void forward (void)         //both wheels forward
{
motion_set(0x06);
}


void back (void)            //both wheels backward
{
motion_set(0x09);
}


void left (void)            //Left wheel backward, Right wheel forward
{
motion_set(0x05);
}


void right (void)           //Left wheel forward, Right wheel backward
{
motion_set(0x0A);
}


void soft_left (void)       //Left wheel stationary, Right wheel forward
{
motion_set(0x04);
}


void soft_right (void)      //Left wheel forward, Right wheel is stationary
{
motion_set(0x02);
}
void soft_left_2 (void)     //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}


void soft_right_2 (void)    //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}


void hard_stop (void)       //hard stop(stop suddenly)
{
  motion_set(0x00);
}


void soft_stop (void)       //soft stop(stops slowly)
{
  motion_set(0x0F);
}


//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;                //ref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;                //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}




void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 adc_init();
 sei(); //Enables the global interrupts
}




//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}


//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;   //set PORTF direction as input
 PORTA = 0x00;  //set PORTF pins floating
}


//Function to Initialize PORTS
void port_init()
{
 lcd_port_config();
 adc_pin_config();
 motion_pin_config();
}


//TIMER1 initialize - rescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFF; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D; //start Timer
}


void velocity (unsigned int left_motor, unsigned int right_motor)
{
         OCR1AL = left_motor;
         OCR1BL = right_motor;
}


//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;
 ADMUX= 0x20| Ch;
 ADCSRA = ADCSRA | 0x40;        //Set start conversion bit
 while((ADCSRA&0x10)==0);        //Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}


void PID_calculator()
{
        l=ADC_Conversion(3);
        c=ADC_Conversion(4);
        r=ADC_Conversion(5);
        // ideal rover position would be straight pura time 
        if(mode == 2)
        {
        P=Kp*e;
        I=I+e;
        I=I*Ki;
        D=(e-pe)*Kd;
        motorspeed = P + I + D;
        if(motorspeed > 55)
        {
                motorspeed = 55;                
        }        
        motorspeeda = basespeed - motorspeed;
        motorspeedb = basespeed + motorspeed;
        forward();
        }
        
        if(mode == 1)
        {
                P=Kp*e;
                I=I+e;
                I=I*Ki;
                D=(e-pe)*Kd;
                motorspeed = P + I + D;
                motorspeed = motorspeed *2;
                if(motorspeed > 55)
                {
                        motorspeed = 55;
                }
                motorspeeda = basespeed + motorspeed;
                motorspeedb = basespeed - motorspeed;
                forward();
        }
        if(mode == 3)
        {


                motorspeed = 0;
                motorspeeda = basespeed ;
                motorspeedb = basespeed ;
                forward();
                
        }
        if(mode == 4)
        {


                motorspeed = 0;
                motorspeeda = basespeed ;
                motorspeedb = basespeed ;
                back();
        }
        
        
}


void errorf()
{
        l=ADC_Conversion(3);
        c=ADC_Conversion(4);
        r=ADC_Conversion(5);
        
        if(c <= 9)
        {
                if(r > 10 && l < 10)
                {
                        e = r ;
                        mode = 1;                        
                }
                if(l > 10 && r < 10 )                                                                                                    
                {
                        e = l;
                        mode = 2;
                }
                if(r < 10 && l < 10)
                {
                        e = 0;
                        mode = 4;
                }
                
        }
        if(c > 10)
        {
                if(r > 10 && l < 10)
                {
                        e = r ;
                        mode = 1;
                }
                if(l > 10 && r < 10 )
                {
                        e = l;
                        mode = 2;
                }
                if(r < 10 && l < 10)
                {
                        e = 0;
                        mode = 3;
                }
                if(l > 10 && r > 10)
                {
                        e = 0;
                        mode = 3;
                }
        }
}


//Main Function
int main(void)
{
 init_devices();
 lcd_set_4bit();
 lcd_init();
 timer1_init();


while(1)
{
        l=ADC_Conversion(3);
        c=ADC_Conversion(4);
        r=ADC_Conversion(5);
        lcd_print(1, 1, motorspeed,3);
        lcd_print(1, 13, r , 3);
        lcd_print(1, 5, l, 3);
        lcd_print(1, 9, c, 3);
        lcd_print(2, 1, mode, 3);
        
        errorf();
        PID_calculator();
        velocity(motorspeeda, motorspeedb);
        
        
}


        


/*
while(1)
{
        
        
        
        forward();            //both wheels forward
        _delay_ms(1000);


        hard_stop();                                                
        _delay_ms(300);


        back();               //both wheels backward                                                
        _delay_ms(1000);


        hard_stop();                                                
        _delay_ms(300);


        left();               //Left wheel backward, Right wheel forward
        _delay_ms(1000);


        hard_stop();                                                
        _delay_ms(300);


        right();              //Left wheel forward, Right wheel backward
        _delay_ms(1000);


        hard_stop();                                                
        _delay_ms(300);


        soft_left();          //Left wheel stationary, Right wheel forward
        _delay_ms(1000);


        hard_stop();                                                
        _delay_ms(300);


        soft_right();         //Left wheel forward, Right wheel is stationary
        _delay_ms(1000);


        hard_stop();                                                
        _delay_ms(300);


        soft_left_2();        //Left wheel backward, right wheel stationary
        _delay_ms(1000);


        hard_stop();                                                
        _delay_ms(300);


        soft_right_2();       //Left wheel stationary, Right wheel backward
        _delay_ms(1000);


        hard_stop();                                                
        _delay_ms(300);
        
}
*/
}