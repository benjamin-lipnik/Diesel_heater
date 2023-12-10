#ifndef CONFIG_H
#define CONFIG_H

#define TRUE        1
#define FALSE       0

////////////////////////////////////////////////////////////////////////////////
//NASTAVITVE DELOVANJA//////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//temperatura v stopinjah
#define MAX_TEMPERATURA 50
#define MIN_TEMPERATURA -5

//Cas v milisekundah
#define CAS_ISKRENJE      3000
#define CAS_VBRIZGAVANJE  5500
#define CAS_OHLAJANJE     90000

#define ST_PONOVNIH_POSKUSKOV_VZIGA 2

//Nizja meja -> Svetleje more biti za uspesen vzig
//Visja meja -> Uspesen vzig v temnejsem okolju
//Nastavi mejo med 5 in 500
#define MEJA_VZIGA_FOTOCELICA 180

#define TERMISTOR_R2               10000
#define TERMISTOR_UPORNOST         10000
#define TERMISTOR_KOEFICIENT       4200
//Ce termocelica ni povezana ali ce je crkjena 5 je vredu vrednost
#define TERMISTOR_MEJA_NAPAKA      5

////////////////////////////////////////////////////////////////////////////////
//KONEC NASTAVITEV//////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

//NE SPREMINJAJ!!!
#define EXTERNAL_OSCILATOR FALSE

#define OUTPUT_PORTB PORTB
#define OUTPUT_DDRB  DDRB
#define OUTPUT_PINB  PINB

#define INPUT_PORTC PORTC
#define INPUT_DDRC  DDRC
#define INPUT_PINC  PINC

#define DISPLAY_PORTD PORTD
#define DISPLAY_DDRD  DDRD
#define DISPLAY_PIND  PIND

#define DISPLAY1_EN_PIN 4
#define DISPLAY2_EN_PIN 3
#define DISPLAY3_EN_PIN 5
#define DISPLAY4_EN_PIN 0

#define DISPLAY123_PORTB PORTB
#define DISPLAY123_DDRB  DDRB
#define DISPLAY123_PINB  PINB

#define DISPLAY4_PORTC   PORTC
#define DISPLAY4_DDRC    DDRC
#define DISPLAY4_PINC    PINC

#define OUTPUT_PIN_VENTIL          0
#define OUTPUT_PIN_ISKRA           1
#define OUTPUT_PIN_VENTILATOR      2

#define INPUT_PIN_ENCODER_1        1
#define INPUT_PIN_ENCODER_2        2
#define INPUT_PIN_LDR              3
#define INPUT_PIN_THERMISTOR       4
#define INPUT_PIN_STIKALO          5

#define LDR_ADC_PIN A3
#define NTC_ADC_PIN A4

#if EXTERNAL_OSCILATOR != TRUE
  #define SETTINGS_TWO_RELAY_PIN 6
  #define SETTINGS_SETTING_TWO_PIN 7

  #define SETTINGS_PORTB PORTB
  #define SETTINGS_DDRB  DDRB
  #define SETTINGS_PINB  PINB
#endif

#define FOTOCELICA_GORI(vrednost) ((vrednost) <= MEJA_VZIGA_FOTOCELICA)
#define STIKALO_ON (!(INPUT_PINC & _BV(INPUT_PIN_STIKALO)))
#define TERMISTOR_PROBLEM(vrednost) ((vrednost) < TERMISTOR_MEJA_NAPAKA)

#define PRINT(x) {}
#define PRINTLN(x) {}

#define DEBUG_MODE FALSE

#if DEBUG_MODE == TRUE
  #define UART_RX_PIN   0
  #define UART_TX_PIN   1
  #define UART_BAUDRATE 9600
  #define PRINT(x) Serial.print(x)
  #define PRINTLN(x) Serial.println(x)
#endif

#endif
