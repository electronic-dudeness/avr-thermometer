/*
(Wireless) AVR thermometer

SMD LED -> gnd is on the left.

ONE_WIRE is PB2

anode 1  PC0
anode 2  PC1
anode 3  PC2

segments:
A PC3
B PC4
C PC5
D PD3
E PD4
F PD5
G PD6
        
nr/seg  A   B   C          D   E   F   G
0       1   1   1          1   1   1   0
1       0   1   1          0   0   0   0
2       1   1   0          1   1   0   1
3       1   1   1          1   0   0   1
4       0   1   1          0   0   1   1
5       1   0   1          1   0   1   1
6       1   0   1          1   1   1   1
7       1   1   1          0   0   0   0
8       1   1   1          1   1   1   1 
9       1   1   1          1   0   1   1
*/                        
  


#ifndef F_CPU
#define F_CPU 1000000UL    // 1MHz clock speed
#endif

#define BAUD 19200
#define MAXSENSORS 1
#define NEWLINESTR "\r\n"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>
//#include <string.h>
//#include <stdint.h>

#include <avr/power.h>
#include <avr/sleep.h>

#include "uart.h"
#include "uart_addon.h"
#include "onewire.h"
#include "ds18x20.h"

#define anodesOff (PORTC &= 0b11111000) //all led displays off
#define LED_ON PORTD |= (1 << PD7)
#define LED_OFF PORTD &= ~(1<< PD7)
#define LED_TOGGLE PORTD ^= (1<< PD7)

#define DEBOUNCETIME 250 //for button interaction
#define CHECKBIT(PORT,BIT) (PORT & (1<<BIT))

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

volatile uint8_t anode=0; //led display that is currently on
volatile uint8_t display[3]; //array that contains numbers to be displayed
volatile uint8_t displayOn, buttonCounter;

static uint8_t search_sensors(void){
  uint8_t i;
  uint8_t id[OW_ROMCODE_SIZE];
  uint8_t diff, nSensors;
  //uart_puts_P( NEWLINESTR "Scanning Bus for DS18X20" NEWLINESTR );
  ow_reset();
  nSensors = 0;
  diff = OW_SEARCH_FIRST;
  while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
    DS18X20_find_sensor( &diff, &id[0] );
    if( diff == OW_PRESENCE_ERR ) {
      //uart_puts_P( "No Sensor found" NEWLINESTR );
      break;
    }
    if( diff == OW_DATA_ERR ) {
      //uart_puts_P( "Bus Error" NEWLINESTR );
      break;
    }
    for ( i=0; i < OW_ROMCODE_SIZE; i++ )
      gSensorIDs[nSensors][i] = id[i];
    nSensors++;
  }
  return nSensors;
}

void displayNumber(uint8_t number){
  //arrays that hold portbits to display numbers:
  //                        0           1            2          3           4           5           6          7             8         9 
  uint8_t segC[10] = { 0b00000000, 0b00001000, 0b00100000, 0b00000000, 0b00001000, 0b00010000, 0b00010000, 0b00000000, 0b00000000, 0b00000000};
  uint8_t segD[10] = { 0b01000000, 0b01111000, 0b00100000, 0b00110000, 0b00011000, 0b00010000, 0b00000000, 0b01111000, 0b00000000, 0b00010000}; 

  PORTC = ((PORTC & 0b11000111) | segC[number]);
  PORTD = ((PORTD & 0b10000111) | segD[number]);
}

void readTemp()
{
  PORTC &= 0b11111000;; //turns off display because interrupts are disabled during DS18b20 measurements
  volatile int16_t decicelsius;
  DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL );
  _delay_ms( DS18B20_TCONV_12BIT );
  DS18X20_read_decicelsius_single( gSensorIDs[0][0], &decicelsius );
  display[0] = decicelsius / 100; 
  display[1] = (decicelsius - (display[0] * 100))/ 10;
  display[2] = decicelsius % 10;
}

ISR (TIMER2_COMP_vect){ //Timer2 for multiplexing
  
  //the button on the side can shut down the display.
  
  buttonCounter++; //stuff with buttonCounter is for debouncing
  if (buttonCounter > 254) //if buttonCounter is about to overflow
    buttonCounter = 51;
  if (!CHECKBIT(PIND, PD0) && buttonCounter > 50){ //if button is pressed
    buttonCounter = 0;
    if (displayOn == 1){
      displayOn = 0;
      PORTC &= 0b11111000;  //all anodes off
    }  
    else
      displayOn = 1;
  }
  if (displayOn == 1){
    anode++;
    if (anode > 2) { anode=0; }
    PORTC &= 0b11111000;  //all anodes off
    displayNumber(display[anode]);
    PORTC |= (1 << anode);  //anode that's up
  }
}

ISR (TIMER1_COMPA_vect){ //Timer1 for reading sensor
  readTemp();
}

int main(void){
  
  DDRD &= ~(1 << PD0); //button
  PORTD |= (1 << PD0); //pullup

  DDRC |= 0b00111111; //anodes + segments a, b and c
  DDRD |= 0b11111000; //segments d, e, f, g and led
    
  anodesOff;
  displayOn = 1; //display is on
      
  /* setup 16 bits timer1 for reading temperature
  with 1024 prescaler running at 1MHz with overflow interrupt
  1MHz / 1024 = 976.5625Hz
  976.56 / 0.1 Hz = 9765.6
  */
    
  OCR1A = 30000; //about once very 30 seconds w00t w00t 16 bits!
  TCCR1B |= (1<<WGM12); //CTC OCR1A
  TCCR1B |= (1<<CS12) | (1<<CS10); //1024 prescale 
  TIMSK = (1 << OCIE1A); //trigger interrupt on overflow

  /* setup timer2 for multiplexing 
  Timer 2 is an 8 bit timer, we'll set it with 1024 prescale with CTC mode 
  on OCR2 = 5;
  */

  OCR2 = 5;
  TCCR2 |= (1<<WGM21); //CTC mode
  TCCR2 |= (1<<CS20) | (1<<CS21) | (1<<CS22); //1024 
  TIMSK |= (1 << OCIE2); //trigger interrupt on overflow
  
  //uart_init((UART_BAUD_SELECT((BAUD),F_CPU)));
  sei(); //turn on interrupts
  
  /*
  for (uint8_t i=0; i<10; i++){
    display[0] = display[1] = display[2] = i;
    _delay_ms(300);
  }
  */
  search_sensors(); //populate gSensorIDs
  readTemp();
  
  while(1){
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_mode();
  }
  return 1; //never reached, but compiler longs for it.
}