#include "config.h"

enum Glavna_stanja {
  STATE_OFF = 0,
  STATE_STARTING,
  STATE_ON,
  STATE_STOPPING,
  STATE_ERROR,
  STATE_WAITING
};

enum Stanja_napake {
  STATE_ERROR_NONE = 0,
  STATE_ERROR_ER1,
  STATE_ERROR_ER2,
  STATE_ERROR_ER3,
  STATE_ERROR_ER4
};

enum Stanja_vziga {
  STATE_STARTING_BEGIN = 0,
  STATE_STARTING_PREHEATING,
  STATE_STARTING_VENTILATING
};

enum Stanja_izklopa {
  STATE_STOPPING_BEGIN = 0,
  STATE_STOPPING_COOLING,
  STATE_STOPPING_SPARKING1,
  STATE_STOPPING_SPARKING2
};

//To bo slo v nek struct state scheduler
uint8_t main_state = STATE_OFF;
uint8_t error_number = STATE_ERROR_NONE;

struct {
  enum Glavna_stanja stanje_prej;
  uint32_t cas_ob_zacetku;
  uint32_t cas_deleya;
}Sleep_data = {STATE_OFF,0,0};

const uint8_t cifre[10] = {
  0b00111111,
  0b00001100,
  0b01011011,
  0b01011101,
  0b01101100,
  0b01110101,
  0b01110111,
  0b00011100,
  0b01111111,
  0b01111101
};
const uint8_t minus = _BV(6);
const uint8_t char_e = 0b01110011;
const volatile int8_t rotacijska_tabela[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

struct {
  volatile int8_t values_on_display[2];
  volatile uint8_t digits_buffer[4];
  volatile uint8_t digit_selector;
  volatile uint8_t common_anode;
}Display_data = {{0,0},{minus,minus,minus,minus},0,0};

struct {
  volatile uint8_t enable;
  volatile int8_t sub_cnt;
  volatile int16_t count;
  volatile uint8_t last_ab;
}Encoder_data = {1,0,20,0};

struct {
  volatile uint16_t ldr_raw;
  volatile uint16_t ntc_raw;
  volatile float ldr_filtered;
  volatile float ldr_min_filtered;
  volatile uint16_t ldr_min;
  volatile uint8_t ldr_min_counter;
  volatile int8_t temp_deg_raw;
  volatile int8_t temp_deg_filtered;
}Sensor_data = {500,500,500,500,500,0,20,20};

char * serial_reader() {
#if DEBUG_MODE == TRUE
  static char serial_reader_buffer[256] = {0};
  static uint8_t serial_reader_index = 0;

  while(Serial.available()) {
    char dat_read = Serial.read();
    if(dat_read == '\n' || dat_read == '\r' || dat_read == 0) {
      if(serial_reader_index == 0)
        return 0;
      serial_reader_buffer[serial_reader_index] = 0;
      serial_reader_index = 0;
      return serial_reader_buffer;
    }
    serial_reader_buffer[serial_reader_index++] = dat_read;
  }

#endif
  return 0;
}
uint8_t nastavi_bite(uint8_t vrednost, uint8_t biti, uint8_t nastavi_na) {
  if(nastavi_na) {
    return vrednost | biti;
  }
  return vrednost & ~biti;
}

void set_display_to_number(int8_t val, uint8_t left) {
  Display_data.values_on_display[left] = val;
  if(val >= 0) {
    Display_data.digits_buffer[left*2+1] = cifre[(val/10)%10] | ((val > 199) << 7);
    Display_data.digits_buffer[left*2]   = cifre[val%10] | ((val > 99) << 7);
  }
  else {
    if(val < -9)
      val = -9;
    Display_data.digits_buffer[left*2+1] = minus;
    Display_data.digits_buffer[left*2]   = cifre[-val];
  }
}
void display_show_error() {
  Display_data.digits_buffer[3] = char_e;
  Display_data.digits_buffer[2] = cifre[error_number];
}

void setup() {
  #if DEBUG_MODE == TRUE
    //NE Moremo uporabljat displaya v debug_nacinu
    Serial.begin(UART_BAUDRATE);
  #else
    //Nastavitve displaya
    DISPLAY_DDRD |= 0xff;
    Display_data.common_anode = DISPLAY4_PINC & _BV(DISPLAY4_EN_PIN); //zmerimo pred tem ko nastavimo pin na output
    DISPLAY4_DDRC |= _BV(DISPLAY4_EN_PIN);
    DISPLAY123_DDRB |= _BV(DISPLAY1_EN_PIN) | _BV(DISPLAY2_EN_PIN) | _BV(DISPLAY3_EN_PIN);

  #endif

  OUTPUT_DDRB |= _BV(OUTPUT_PIN_ISKRA) | _BV(OUTPUT_PIN_VENTIL) | _BV(OUTPUT_PIN_VENTILATOR);
  INPUT_PORTC |= _BV(INPUT_PIN_ENCODER_1) | _BV(INPUT_PIN_ENCODER_2); //PULLUPI

  //PIN CHANGE INTERRUPT ZA ENCODER
  PCICR |= _BV(PCIE1);
  PCMSK1 |= _BV(PCINT9) | _BV(PCINT10);

  //TIMER, COUNTER, INTERRUPT ZA DISPLAY
  TCCR1A = 0;
  TCCR1B = _BV(WGM13) | _BV(WGM12) | 2; //clk 1Mhz
  TIMSK1 |= _BV(TOIE1) | _BV(ICIE1);
  ICR1 = 4000; //display refresh rate f = 250Hz
  sei();

  analogReference(EXTERNAL);

  delay(50);
  Sensor_data.ntc_raw = analogRead(NTC_ADC_PIN);
  delay(50);
  //set_display_to_number(Encoder_data.count, 1);

  PRINTLN("OK");
}

int8_t calculate_temp() {
  float voltage = 5.0f * ((float)Sensor_data.ntc_raw / 1023.0f);
  uint32_t r_ntc = 0.5f + (float)(TERMISTOR_R2) * ((float)(5.0f - voltage) / (float)voltage);

  float steinhart;
  steinhart = (float)r_ntc / (float)(TERMISTOR_UPORNOST);     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= (float)(TERMISTOR_KOEFICIENT);                   // 1/B * ln(R/Ro)
  steinhart += 1.0f / (25.0f + 273.15f); // + (1/To)
  steinhart = 1.0f / steinhart;                 // Invert
  steinhart -= 273.15f;

  if(steinhart > 99.0f)
    steinhart = 99.5f;
  if(steinhart < -9.0f)
    steinhart = -9.5f;

  return steinhart;
}


/////ROUTINE ZA STANJA/////

void off_state() {
  //gledam kdaj pade temperatura pod zeljeno + da je gumb prizgan
  //if(LDR_PROBLEM) {
  //  main_state = STATE_ERROR;
  //  error_number = STATE_ERROR_ER3;
  //    return;
  //}
  if(TERMISTOR_PROBLEM(Sensor_data.ntc_raw) || (Sensor_data.temp_deg_filtered > 60)) {
    main_state = STATE_ERROR;
    error_number = STATE_ERROR_ER2;
    return;
  }

  if(!STIKALO_ON)
    return;

  if(Sensor_data.temp_deg_filtered < Encoder_data.count) {
    main_state = STATE_STARTING;
  }

  if(OUTPUT_PORTB & (_BV(OUTPUT_PIN_VENTIL) | _BV(OUTPUT_PIN_VENTIL) | _BV(OUTPUT_PIN_VENTILATOR))) {
    OUTPUT_PORTB &= ~(_BV(OUTPUT_PIN_VENTIL) | _BV(OUTPUT_PIN_VENTIL) | _BV(OUTPUT_PIN_VENTILATOR));
    main_state = STATE_ERROR;
    error_number = STATE_ERROR_ER4;
  }
}
void starting_state() {
  static uint8_t starting_state = STATE_STARTING_BEGIN;
  static uint8_t retry_cnt = 0;

  PRINT("VZIG ");
  PRINT(starting_state);
  PRINT("\n");

  if(starting_state == STATE_STARTING_BEGIN) {
    //NOPE//izklopi ventil, ventilator, ce sta slucajno prizgana
    //OUTPUT_PORTB &= ~(_BV(OUTPUT_PIN_VENTIL) | _BV(OUTPUT_PIN_VENTILATOR));
    //1. Vklopi iskro/grelec aka output 1
    OUTPUT_PORTB |= _BV(OUTPUT_PIN_ISKRA);

    starting_state = STATE_STARTING_PREHEATING;
    set_wait(CAS_ISKRENJE);
  }
  else if(starting_state == STATE_STARTING_PREHEATING) {
    //2. Vklopi ventilator in ventil
    OUTPUT_PORTB |= _BV(OUTPUT_PIN_VENTIL) | _BV(OUTPUT_PIN_VENTILATOR);
    starting_state = STATE_STARTING_VENTILATING;
    set_wait(CAS_VBRIZGAVANJE);
  }
  else if(starting_state == STATE_STARTING_VENTILATING) {
    //izklopi grelec/iskro
    OUTPUT_PORTB &= ~_BV(OUTPUT_PIN_ISKRA);

    //Poglej ce se ne gori.
    if(!FOTOCELICA_GORI(Sensor_data.ldr_filtered)) {
      if(retry_cnt++ < ST_PONOVNIH_POSKUSKOV_VZIGA) {
        //Se enkrat proba
        //izklopi vse
        OUTPUT_PORTB &= ~(_BV(OUTPUT_PIN_VENTIL) | _BV(OUTPUT_PIN_VENTILATOR));
        PRINT("Neuspel vzig, ponovni poiskus.\n");
        set_wait(500);
        starting_state = STATE_STARTING_BEGIN;
        return;
      }
      else { //CE NE GORI PO STARTING_RETRIES POSKUSIH
        main_state = STATE_ERROR;
        error_number = STATE_ERROR_ER1;
        PRINT("Neuspel vzig!\n");
      }
    }
    else { //Poglej ce zdej ze gori
      main_state = STATE_ON;
      PRINT("Prizgan ok\n");
    }

    retry_cnt = 0;
    starting_state = STATE_STARTING_BEGIN;//resitirmo sub_state_machine
  }
}
void on_state() {
  //gledam ce je temperatura ze prerasla zeljeno + ce se prtisne gumb za ugasnat
  if(Sensor_data.temp_deg_filtered >= Encoder_data.count) {
    PRINT("Temperatura dosezena, ustavljanje\n");
    main_state = STATE_STOPPING;
    return;
  }
  if(!STIKALO_ON) {
    PRINT("STIKALO!, ustavljanje\n");
    main_state = STATE_STOPPING;
    return;
  }
  //gledam ce fotocelcia zazna temno, kar pomeni da je plamen zgasno
  if(!FOTOCELICA_GORI(Sensor_data.ldr_min_filtered)) {
    //probam ponovni vzig
    PRINT("FOTOCELICA, ponovni vzig\n");
    main_state = STATE_STARTING;
    //main_state = STATE_ERROR;
    //error_number = STATE_ERROR_ER1;
  }
  if(TERMISTOR_PROBLEM(Sensor_data.ntc_raw)) {
    PRINT("Napaka termistorja\n");
    main_state = STATE_ERROR;
    error_number = STATE_ERROR_ER2;
  }
}
void stopping_state() {
  static uint8_t stopping_state = STATE_STOPPING_BEGIN;

  PRINT("USTAVLJANJE ");
  PRINT(stopping_state);
  PRINT("\n");

  if(stopping_state == STATE_STOPPING_BEGIN) {
    //vklopi iskro, da se ne kadi
    OUTPUT_PORTB |= _BV(OUTPUT_PIN_ISKRA);
    stopping_state = STATE_STOPPING_SPARKING1;
    set_wait(1000);
  }
  else if(stopping_state == STATE_STOPPING_SPARKING1) {
    //izklopi ventil
    OUTPUT_PORTB &= ~_BV(OUTPUT_PIN_VENTIL);
    stopping_state = STATE_STOPPING_SPARKING2;
    set_wait(1000);
  }
  else if(stopping_state == STATE_STOPPING_SPARKING2) {
    //izklopi iskro
    OUTPUT_PORTB &= ~_BV(OUTPUT_PIN_ISKRA);
    stopping_state = STATE_STOPPING_COOLING;
    set_wait(CAS_OHLAJANJE);
  }
  else if(stopping_state == STATE_STOPPING_COOLING) {
    //izklopim ventilator
    OUTPUT_PORTB &= ~_BV(OUTPUT_PIN_VENTILATOR);
    main_state = STATE_OFF;
    stopping_state = STATE_STOPPING_BEGIN;
  }
}
void error_state() {
  static uint8_t error_substate = 0;

  PRINT("ERROR ");
  PRINT(error_number);
  PRINT("\n");

  Encoder_data.enable = 0;
  display_show_error();

  if(error_substate == 0) {
    //izklopi ventil in iskro/grelec
    OUTPUT_PORTB &= ~(_BV(OUTPUT_PIN_ISKRA) | _BV(OUTPUT_PIN_VENTIL));
    set_wait(CAS_OHLAJANJE);
    error_substate++;
  }
  else if(error_substate == 1) {
    OUTPUT_PORTB &= ~_BV(OUTPUT_PIN_VENTILATOR); //izklopi ventilator

    while((error_number != STATE_ERROR_ER1) || STIKALO_ON) { //SAM IZ ERROR_1 LAHKA GRES VEN
      delay(10);
      display_show_error();
    }

    Encoder_data.enable = 1;
    main_state = STATE_OFF;
    set_display_to_number(Encoder_data.count, 1);
    error_number = 0;
    error_substate = 0;
  }
}
void waiting_state() {
  if(millis() - Sleep_data.cas_ob_zacetku < Sleep_data.cas_deleya) //ce je delay aktiven
    return;
  if(Sleep_data.stanje_prej == STATE_WAITING) {
    Sleep_data.stanje_prej = STATE_ERROR;
  }
  main_state = Sleep_data.stanje_prej;
  Sleep_data.cas_deleya = 0;
}
void set_wait(uint32_t wait_ms) {
  Sleep_data.stanje_prej = main_state;
  Sleep_data.cas_ob_zacetku = millis();
  Sleep_data.cas_deleya = wait_ms;
  main_state = STATE_WAITING;
}

const void (* main_state_calls_table[])(void) = {off_state, starting_state, on_state, stopping_state, error_state, waiting_state};
//////////////////////////

void loop() {

  main_state_calls_table[main_state]();

#if DEBUG_MODE == TRUE

  static uint8_t izpis = 1;

  char tx_buff[256] = {0};
  char * str = serial_reader();
  if(str) {
    if(strcmp(str, "IZPIS") == 0) {
      izpis = !izpis;
    }
    else if(strcmp(str, "OFF") == 0) {
      main_state = STATE_STOPPING;
    }
    else if(strcmp(str, "ON") == 0) {
      main_state = STATE_STARTING;
    }
  }
  else if(izpis) {
    sprintf(tx_buff, "[%02d] [%02d] ldrf %03d ldrr %03d min %03d", Display_data.values_on_display[1], Display_data.values_on_display[0], (uint16_t)Sensor_data.ldr_filtered, (uint16_t)Sensor_data.ldr_raw, (uint16_t)Sensor_data.ldr_min_filtered);
    PRINTLN(tx_buff);
  }
#endif

}

//ENCODER...
ISR(PCINT1_vect) {
  uint8_t ab = !(INPUT_PINC & _BV(INPUT_PIN_ENCODER_1)) << 1 | !(INPUT_PINC & _BV(INPUT_PIN_ENCODER_2));
    Encoder_data.sub_cnt += rotacijska_tabela[Encoder_data.last_ab << 2 | ab];
    if((abs(Encoder_data.sub_cnt) >= 4) && Encoder_data.enable && STIKALO_ON) {
      int8_t new_cnt = Encoder_data.count + ((Encoder_data.sub_cnt > 0)*2-1);
      Encoder_data.sub_cnt = 0;
      if(new_cnt > MAX_TEMPERATURA)
        Encoder_data.count = MAX_TEMPERATURA;
      else if(new_cnt < MIN_TEMPERATURA)
        Encoder_data.count = MIN_TEMPERATURA;
      else
        Encoder_data.count = new_cnt;
      //display updatam c v drugem isrju tak da tu ni treba
      //set_display_to_number(Encoder_data.count, 1);
      //set_display_4dig(Encoder_data.count);
    }
    Encoder_data.last_ab = ab;
}

//DISPLAY, ANALOG READS
ISR(TIMER1_CAPT_vect) {
  if(++Display_data.digit_selector >= 4)
    Display_data.digit_selector = 0;

#if DEBUG_MODE != TRUE
  //ugasne vse displaye
  //ce je CA pol da NE svetijo das na 1, ce je CC pol da NE svetijo das na 0
  DISPLAY123_PORTB = nastavi_bite(DISPLAY123_PORTB, _BV(DISPLAY1_EN_PIN) | _BV(DISPLAY2_EN_PIN) | _BV(DISPLAY3_EN_PIN), Display_data.common_anode);
  DISPLAY4_PORTC = nastavi_bite(DISPLAY4_PORTC, _BV(DISPLAY4_EN_PIN), Display_data.common_anode);

  //prestavi na drugo stevilko med tem ko so displayi ugasnjeni
  if(Display_data.common_anode)
    DISPLAY_PORTD = ~Display_data.digits_buffer[Display_data.digit_selector];
  else
    DISPLAY_PORTD = Display_data.digits_buffer[Display_data.digit_selector];

  switch(Display_data.digit_selector) {
    case 0:
      DISPLAY123_PORTB = nastavi_bite(DISPLAY123_PORTB, _BV(DISPLAY2_EN_PIN), !Display_data.common_anode);
      break;
    case 1:
      DISPLAY123_PORTB = nastavi_bite(DISPLAY123_PORTB, _BV(DISPLAY1_EN_PIN), !Display_data.common_anode);
      break;
    case 2:
      DISPLAY4_PORTC = nastavi_bite(DISPLAY4_PORTC, _BV(DISPLAY4_EN_PIN), !Display_data.common_anode);
      break;
    case 3:
      DISPLAY123_PORTB = nastavi_bite(DISPLAY123_PORTB, _BV(DISPLAY3_EN_PIN), !Display_data.common_anode);
      break;
  }
#endif

  set_display_to_number(Sensor_data.temp_deg_filtered, 0);
  if(main_state != STATE_ERROR) {
    if(STIKALO_ON) {
      set_display_to_number(Encoder_data.count, 1);
    }
    else {
      Display_data.digits_buffer[2] = minus;
      Display_data.digits_buffer[3] = minus;
    }
  }

  Sensor_data.ldr_raw = analogRead(LDR_ADC_PIN);
  Sensor_data.ntc_raw = analogRead(NTC_ADC_PIN);

  if(Sensor_data.ldr_raw < Sensor_data.ldr_filtered) {
    Sensor_data.ldr_filtered = Sensor_data.ldr_filtered * 0.99f + (float)Sensor_data.ldr_raw * 0.01f;
  }
  else {
    Sensor_data.ldr_filtered = Sensor_data.ldr_filtered * 0.995f + (float)Sensor_data.ldr_raw * 0.005f;
  }
  if(++Sensor_data.ldr_min_counter >= 120) {
    Sensor_data.ldr_min_counter = 0;
    Sensor_data.ldr_min_filtered = Sensor_data.ldr_min_filtered * 0.4f + Sensor_data.ldr_min * 0.6f;
    Sensor_data.ldr_min = 0xffff;
  }
  if(Sensor_data.ldr_raw < Sensor_data.ldr_min) {
    Sensor_data.ldr_min = Sensor_data.ldr_raw;
  }

  Sensor_data.temp_deg_raw = calculate_temp();

  Sensor_data.temp_deg_filtered = Sensor_data.temp_deg_filtered * 0.625f + (float)Sensor_data.temp_deg_raw * 0.375f;
}
