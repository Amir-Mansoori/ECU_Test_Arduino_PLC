/* CAN Parameters */
#define CAN_2515
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

unsigned char live = 1;
uint32_t id;
uint8_t  len;
uint8_t err_ptr = 0;
byte cdata[8] = {0};

// Pin connected to the PWM output
const int pwmPin1 = 11; // OC1A
const int pwmPin2 = 5; // OC3A
const int pwmPin3 = 6; // OC4A
const int pwmPin4 = 46; // OC5A
// Variables to hold frequency and duty cycle values
int Flex_PWM4_Frequency;  // frequency in Hz
float Flex_PWM4_DC;  // duty cycle in percentage (0-100)

int Flex_PWM2_Frequency;  // frequency in Hz
float Flex_PWM2_DC;  // duty cycle in percentage (0-100)

int PYRO_Crash_pwm_Frequency;  // frequency in Hz
float PYRO_Crash_pwm_DC;  // duty cycle in percentage (0-100)

int Flex_PWM5_Frequency;  // frequency in Hz
float Flex_PWM5_DC;  // duty cycle in percentage (0-100)


/* CAN nessages for PWM Generator Signals */
uint8_t PYRO_Crash_Freq_raw[2];
uint16_t PYRO_Crash_Freq_value_raw;
uint8_t PYRO_Crash_DC_raw;

uint8_t Flex_PWM2_Freq_raw[2];
uint16_t Flex_PWM2_Freq_value_raw;
uint8_t Flex_PWM2_DC_raw;

uint8_t Flex_PWM4_Freq_raw[2];
uint16_t Flex_PWM4_Freq_value_raw;
uint8_t Flex_PWM4_DC_raw;

uint8_t Flex_PWM5_Freq_raw[2];
uint16_t Flex_PWM5_Freq_value_raw;
uint8_t Flex_PWM5_DC_raw;

void setup() {
  
  // Configure PWM pin as output
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);
  
  // Initialize Timer1 for PWM generation
  // Timer1 is a 16-bit timer available on Arduino Uno
  TCCR1A = 0;  // Clear register A configurations
  TCCR1B = 0;  // Clear register B configurations
  
  TCCR3A = 0;  // Clear register A configurations
  TCCR3B = 0;  // Clear register B configurations
  
  TCCR4A = 0;  // Clear register A configurations
  TCCR4B = 0;  // Clear register B configurations

  TCCR5A = 0;  // Clear register A configurations
  TCCR5B = 0;  // Clear register B configurations    

   SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(50);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
 
}

void loop() {

    CAN.checkError(err_ptr);
    delay(10);
    readCan();
    delay(10); 
    imAlive();
 //   writeCan();
    delay(10);   
  // Example: change frequency and duty cycle over time
 
  // Change frequency (example: 2kHz)




  // Keep loop running
  //while (1);
}

void imAlive(){
  CAN.sendMsgBuf(0x24, 0, 3, live);
  delay(50);
  live +=1;
  if (live >= 65535)live = 0;
}

void readCan(){
   if (CAN_MSGAVAIL != CAN.checkReceive()) {
    
        return;
    }
    CAN.readMsgBuf(&len, cdata);
    id = CAN.getCanId();
    // Serial.println(id);
   if(id==0x20){

    PYRO_Crash_Freq_raw[0] = cdata[0];
    PYRO_Crash_Freq_raw[1] = cdata[1];
    PYRO_Crash_DC_raw = cdata[2];
    PYRO_Crash_Freq_value_raw = (PYRO_Crash_Freq_raw[1] << 8)|(PYRO_Crash_Freq_raw[0]);

    Flex_PWM2_Freq_raw[0] = cdata[3];
    Flex_PWM2_Freq_raw[1] = cdata[4];
    Flex_PWM2_DC_raw = cdata[5];
    Flex_PWM2_Freq_value_raw = (Flex_PWM2_Freq_raw[1] << 8)|(Flex_PWM2_Freq_raw[0]);
    
   
    PYRO_Crash_pwm_Frequency = int(PYRO_Crash_Freq_value_raw);
    PYRO_Crash_pwm_DC = float(PYRO_Crash_DC_raw);  
    
    Flex_PWM2_Frequency = int(Flex_PWM2_Freq_value_raw);
    Flex_PWM2_DC = float(Flex_PWM2_DC_raw);

 
    if(PYRO_Crash_pwm_Frequency && PYRO_Crash_pwm_DC)
      pwm_Crash_generate();
    else
      pwm_Crash_off();
    if(Flex_PWM2_Frequency && Flex_PWM2_DC)
      pwm_FlexPWM2_generate();
    else
      pwm_FlexPWM2_off();


    
   }
   if(id==0x23) {

    Flex_PWM4_Freq_raw[0] = cdata[0];
    Flex_PWM4_Freq_raw[1] = cdata[1];
    Flex_PWM4_DC_raw = cdata[2];
    Flex_PWM4_Freq_value_raw = (Flex_PWM4_Freq_raw[1] << 8)|(Flex_PWM4_Freq_raw[0]);

    Flex_PWM5_Freq_raw[0] = cdata[3];
    Flex_PWM5_Freq_raw[1] = cdata[4];
    Flex_PWM5_DC_raw = cdata[5];
    Flex_PWM5_Freq_value_raw = (Flex_PWM5_Freq_raw[1] << 8)|(Flex_PWM5_Freq_raw[0]);
    
   
    Flex_PWM4_Frequency = int(Flex_PWM4_Freq_value_raw);
    Flex_PWM4_DC = float(Flex_PWM4_DC_raw);  
    
    Flex_PWM5_Frequency = int(Flex_PWM5_Freq_value_raw);
    Flex_PWM5_DC = float(Flex_PWM5_DC_raw);

 
    if(Flex_PWM4_Frequency && Flex_PWM4_DC)
      pwm_FlexPWM4_generate();
    else
      pwm_FlexPWM4_off();
    if(Flex_PWM5_Frequency && Flex_PWM5_DC)
      pwm_FlexPWM5_generate();
    else
      pwm_FlexPWM5_off();


    
   }

}


void pwm_Crash_generate(){

  TCCR4A |= (1 << WGM41);
  TCCR4B |= (1 << WGM42) | (1 << WGM43);
 // Set non-inverting mode for PWM output on OC4A (Pin 6 on Arduino MEGA)
  TCCR4A |= (1 << COM4A1);
  TCCR4B |= (1 << CS40);  // Prescaler 1
  //PYRO_Crash_pwm_Frequency = FREQ_3;
//  PYRO_Crash_pwm_DC = DC_3;
  uint16_t top3 = F_CPU / (1UL * PYRO_Crash_pwm_Frequency) - 1;
  uint16_t duty3 = (PYRO_Crash_pwm_DC / 100.0) * (top3 + 1);
  ICR4 = top3;
  OCR4A = duty3;  


}

void pwm_FlexPWM2_generate(){

  TCCR3A |= (1 << WGM31);
  TCCR3B |= (1 << WGM32) | (1 << WGM33);
  // Set non-inverting mode for PWM output on OC3A (Pin 5 on Arduino MEGA)
  TCCR3A |= (1 << COM3A1);  
  TCCR3B |= (1 << CS30);  // Prescaler 1
//  Flex_PWM2_Frequency = FREQ_2;
//  Flex_PWM2_DC = DC_2;
  uint16_t top2 = F_CPU / (1UL * Flex_PWM2_Frequency) - 1;  
  uint16_t duty2 = (Flex_PWM2_DC / 100.0) * (top2 + 1);
  ICR3 = top2;  
  OCR3A = duty2;


}

void pwm_FlexPWM4_generate(){
  // Set mode 14 (Fast PWM, ICR1 is TOP)
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  // Set non-inverting mode for PWM output on OC1A (Pin 11 on Arduino MEGA)
  TCCR1A |= (1 << COM1A1);  
  // Start timer with prescaler 64
  TCCR1B |= (1 << CS10);  // Prescaler 1
//  Flex_PWM4_Frequency = FREQ_1;
//  Flex_PWM4_DC = DC_1;    
  uint16_t top1 = F_CPU / (1UL * Flex_PWM4_Frequency) - 1; 
  uint16_t duty1 = (Flex_PWM4_DC / 100.0) * (top1 + 1);   
  ICR1 = top1;  
  OCR1A = duty1;  
}

void pwm_FlexPWM5_generate(){

  TCCR5A |= (1 << WGM51);
  TCCR5B |= (1 << WGM52) | (1 << WGM53);    
  // Set non-inverting mode for PWM output on OC4A (Pin 46 on Arduino MEGA)
  TCCR5A |= (1 << COM5A1);    
  TCCR5B |= (1 << CS50);  // Prescaler 1    
//  Flex_PWM5_Frequency = FREQ_4;    
//  Flex_PWM5_DC = DC_4;  
  uint16_t top4 = F_CPU / (1UL * Flex_PWM5_Frequency) - 1;  
  uint16_t duty4 = (Flex_PWM5_DC / 100.0) * (top4 + 1);  
  ICR5 = top4;  
  OCR5A = duty4;  


}


void pwm_Crash_off(){
  TCCR4A = 0;  // Clear register A configurations
  TCCR4B = 0;  // Clear register B configurations
  ICR4 = 0;
  OCR4A = 0;
}
void pwm_FlexPWM2_off(){
  TCCR3A = 0;  // Clear register A configurations
  TCCR3B = 0;  // Clear register B configurations
  ICR3 = 0;
  OCR3A = 0;  
}

void pwm_FlexPWM4_off(){
  TCCR1A = 0;  // Clear register A configurations
  TCCR1B = 0;  // Clear register B configurations
  ICR1 = 0;
  OCR1A = 0;  
}

void pwm_FlexPWM5_off(){
  TCCR5A = 0;  // Clear register A configurations
  TCCR5B = 0;  // Clear register B configurations
  ICR5 = 0;
  OCR5A = 0;  
}

// END FILE
