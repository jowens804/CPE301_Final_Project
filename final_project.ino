/*
Jack Owens
Andrew Hsu
Alex Danamidis
Kaitlyn Clevidence
*/

#include <dht.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Stepper.h>

#define RDA 0x80
#define TBE 0x20

dht DHT;
#define DHT11_PIN 3
#define TEMP_THRESHOLD 24

#define START_STOP_PIN 19
#define RESET_PIN 18

#define WATER_THRESHOLD 200

RTC_DS1307 myrtc;

const int RS=53, EN=51, D4=49, D5=47, D6=45, D7=43;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

const int IN1=23, IN2=25, IN3=27, IN4=29;
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;

volatile unsigned char *portA     = (unsigned char *) 0x22;
volatile unsigned char *portDDRA  = (unsigned char *) 0x21;
volatile unsigned char *pinA      = (unsigned char *) 0x20;

volatile unsigned char *portB     = (unsigned char *) 0x25;
volatile unsigned char *portDDRB  = (unsigned char *) 0x24;

volatile unsigned char *portC     = (unsigned char *) 0x28;
volatile unsigned char *portDDRC  = (unsigned char *) 0x27;
volatile unsigned char *pinC      = (unsigned char *) 0x26;

volatile unsigned char *portD     = (unsigned char *) 0x2B;
volatile unsigned char *portDDRD  = (unsigned char *) 0x2A;
volatile unsigned char *pinD      = (unsigned char *) 0x29;

volatile unsigned char *portE     = (unsigned char *) 0x2E;
volatile unsigned char *portDDRE  = (unsigned char *) 0x2D;
volatile unsigned char *pinE      = (unsigned char *) 0x2C;

volatile unsigned char *portF     = (unsigned char *) 0x31;
volatile unsigned char *portDDRF  = (unsigned char *) 0x30;
volatile unsigned char *pinF      = (unsigned char *) 0x2F;

volatile unsigned char *portG     = (unsigned char *) 0x34;
volatile unsigned char *portDDRG  = (unsigned char *) 0x33;
volatile unsigned char *pinG      = (unsigned char *) 0x32;

volatile unsigned char *portH     = (unsigned char *) 0x102;
volatile unsigned char *portDDRH  = (unsigned char *) 0x101;
volatile unsigned char *pinH      = (unsigned char *) 0x100;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

int state, nextState;
char stateNames[4][9] = {"Disabled", "Idle", "Running", "Error"};
int waterLevel;

void setup() {
  *portDDRB |= 0b11110000;
  *portB &= 0b00001111;

  *portDDRA &= 0b11111110;
  *portA &= 0b11111110;

  *portDDRA |= 0b00000100;
  *portA &= 0b11111011;

  *portDDRC &=0b01110111;
  *portC &= 0b01110111;

  *portDDRD &= 0b00000100;
  *portD &= 0b11111011;

  U0Init(9600);

  state = 0;
  nextState = 0;

  attachInterrupt(digitalPinToInterrupt(START_STOP_PIN), startStopButton, RISING);
  attachInterrupt(digitalPinToInterrupt(RESET_PIN), resetButton, RISING);

  myrtc.begin();
  myrtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.write("working");

  adc_init();

  myStepper.setSpeed(5);
}

void loop() {
  if(state != nextState){
    reportTransition();
    if(nextState == 2){
      reportFanOn();
    }else if(state == 2){
      reportFanOff();
    }
  }
  state = nextState;
  if(state == 0){
    write_pb(7, 1);
    write_pb(6, 0);
    write_pb(5, 0);
    write_pb(4, 0);

    *portA &= 0b11111011;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Disabled");
  }else if (state == 1){
    write_pb(7, 0);
    write_pb(6, 1);
    write_pb(5, 0);
    write_pb(4, 0);

    *portA &= 0b11111011;

    int chk = DHT.read11(DHT11_PIN);
    displayTempHumidity();

    waterLevel = adc_read(0);

    while(*pinC & 0b10000000){
      reportVentUp();
      myStepper.step(100);
    }

    while(*pinC & 0b00001000){
      reportVentDown();
      myStepper.step(-100);
    }

    if(waterLevel <= WATER_THRESHOLD){
      nextState = 3;

    }else if(DHT.temperature > TEMP_THRESHOLD){
      nextState = 2;
    }
  }else if(state == 2){
    write_pb(7, 0);
    write_pb(6, 0);
    write_pb(5, 1);
    write_pb(4, 0);

    *portA |= 0b00000100;

    int chk = DHT.read11(DHT11_PIN);
    displayTempHumidity();

    while(*pinC & 0b10000000){
      reportVentUp();
      myStepper.step(100);
    }

    while(*pinC & 0b00001000){
      reportVentDown();
      myStepper.step(-100);
    }

    waterLevel = adc_read(0);

    if(waterLevel < WATER_THRESHOLD){
      nextState = 3;

    }else if(DHT.temperature <= TEMP_THRESHOLD){
      nextState = 1;
    }
  }else if (state == 3){
    write_pb(7, 0);
    write_pb(6, 0);
    write_pb(5, 0);
    write_pb(4, 1);

    *portA &= 0b11111011;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("ERROR: Water");
    lcd.setCursor(0,1);
    lcd.print("level is too low");

    while(*pinC & 0b10000000){
      reportVentUp();
      myStepper.step(100);
    }

    while(*pinC & 0b00001000){
      reportVentDown();
      myStepper.step(-100);
    }
  }
  my_delay(1);
}

void U0Init(int U0baud){
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char kbhit(){
  return *myUCSR0A & RDA;
}

unsigned char getChar(){
  return *myUDR0;
}
void putChar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void write_pb(unsigned char pin_num, unsigned char state){
  if(state == 0){
    *portB &= ~(0x01 << pin_num);
  }
  else{
    *portB |= 0x01 << pin_num;
  }
}

void reportTime(){
  DateTime now = myrtc.now();
  putChar(0x30 + (now.hour()/10)%10);
  putChar(0x30 + now.hour()%10);
  putChar(':');
  putChar(0x30 + (now.minute()/10)%10);
  putChar(0x30 + now.minute()%10);
  putChar(':');
  putChar(0x30 + (now.second()/10)%10);
  putChar(0x30 + now.second()%10);
}

void reportTransition(){
  const char string1[] = "Transition from state ";
  const char string2[] = " to state ";
  const char string3[] = " at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  putChar(0x30 + state);
  for(int i =0; i < strlen(string2); i++ ) {
    char c = string2[i];
    putChar(c);
  }
  putChar(0x30 + nextState);
  for(int i =0; i < strlen(string3); i++ ) {
    char c = string3[i];
    putChar(c);
  }
  reportTime();
  putChar('\n');
}

void reportVentUp(){
  const char string1[] = "Moving vent up at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  reportTime();
  putChar('\n');
}

void reportVentDown(){
  const char string1[] = "Moving vent down at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  reportTime();
  putChar('\n');
}

void reportFanOn(){
  const char string1[] = "Fan turned on at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  reportTime();
  putChar('\n');
}

void reportFanOff(){
  const char string1[] = "Fan turned off at time ";
  for(int i =0; i < strlen(string1); i++ ) {
    char c = string1[i];
    putChar(c);
  }
  reportTime();
  putChar('\n');
}

void startStopButton(){
  if(state == 0){
    nextState = 1;
  }else{
    nextState = 0;
  }
}

void resetButton(){
  waterLevel = adc_read(0);
  if(state == 3 && waterLevel > WATER_THRESHOLD){
    nextState = 1;
  }
}

void displayTempHumidity(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(DHT.humidity);
  lcd.print("%");
}

void adc_init(){
  *my_ADCSRA |= 0b10000000;
  *my_ADCSRA &= 0b11011111;
  *my_ADCSRA &= 0b11110111;
  *my_ADCSRA &= 0b11111000;
  *my_ADCSRB &= 0b11110111;
  *my_ADCSRB &= 0b11111000;
  *my_ADMUX  &= 0b01111111;
  *my_ADMUX  |= 0b01000000;
  *my_ADMUX  &= 0b11011111;
  *my_ADMUX  &= 0b11100000;
}

unsigned int adc_read(unsigned char adc_channel_num){
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;
  if(adc_channel_num > 7){
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

void my_delay(unsigned int freq){
  double period = 1.0/double(freq);
  double half_period = period/ 2.0f;
  double clk_period = 0.0000000625;
  unsigned int ticks = half_period / clk_period;
  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);
  * myTCCR1A = 0x0;
  * myTCCR1B |= 0b00000001;
  while((*myTIFR1 & 0x01)==0);
  *myTCCR1B &= 0xF8;     
  *myTIFR1 |= 0x01;
}