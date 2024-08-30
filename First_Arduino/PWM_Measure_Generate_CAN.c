#define CAN_2515
#define N_CHANNELS 8
#define TIMEOUT 5000
#define N_AVERAGE 50
#define pulse_ip1_A 3
#define pulse_ip1_B 4
#define pulse_ip2_A 5
#define pulse_ip2_B 7
#define pulse_ip3_A 8
#define pulse_ip3_B 10
#define pulse_ip4_A 12
#define pulse_ip4_B 13



const int pwmPin1 = 11; // OC1A
const int pwmPin2 = 6; // OC4A

const int pulse_ip[N_CHANNELS] = {pulse_ip1_A, pulse_ip1_B, pulse_ip2_A, pulse_ip2_B ,pulse_ip3_A, pulse_ip3_B, pulse_ip4_A, pulse_ip4_B};

float ontime[N_CHANNELS],offtime[N_CHANNELS], duty[N_CHANNELS];
float freq[N_CHANNELS]={0}, period[N_CHANNELS] = {0};
///Moving Average parameters
float SUM_ON[N_CHANNELS] = {0};
float SUM_OFF[N_CHANNELS] = {0};
float ON_READINGS[N_CHANNELS][N_AVERAGE] = {0};
float OFF_READINGS[N_CHANNELS][N_AVERAGE] = {0};
int INDEX_ON[N_CHANNELS] = {0};
int INDEX_OFF[N_CHANNELS] = {0};
float AVERAGED_ON_TIME[N_CHANNELS] = {0};
float AVERAGED_OFF_TIME[N_CHANNELS] = {0};

// Variables to hold frequency and duty cycle values
int pwmDistanceFreq;  // frequency in Hz
float pwmDistanceDC;  // duty cycle in percentage (0-100)
int pwmSpeedFreq;  // frequency in Hz
float pwmSpeedDC;  // duty cycle in percentage (0-100)
byte cdata[8] = {0};

const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;

/* CAN messages for PWM Generator Signals */
uint8_t Speed_Pulse_Freq[2];
uint16_t Speed_Pulse_Freq_value;
uint8_t Speed_Pulse_DC;
uint8_t Distance_Signal_Freq[2];
uint16_t Distance_Signal_Freq_value;
uint8_t Distance_Signal_DC;

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

unsigned char live = 1;
uint32_t id;
uint8_t  len;
uint8_t err_ptr = 0;

void setup() {



   pinMode(pwmPin1, OUTPUT);
   pinMode(pwmPin2, OUTPUT);
   // Initialize Timer1 for PWM generation
  // Timer1 is a 16-bit timer available on Arduino Uno
  TCCR1A = 0;  // Clear register A configurations
  TCCR1B = 0;  // Clear register B configurations

  // Timer4 is a 16-bit timer available on Arduino Uno
  TCCR4A = 0;  // Clear register A configurations
  TCCR4B = 0;  // Clear register B configurations
  

   for(int i = 0; i< N_CHANNELS; i++){
     pinMode(pulse_ip[i],INPUT_PULLUP); 
   }
    
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
    pwm_measure();
    readCan();
    delay(10); 
    imAlive();
    writeCan();
    delay(10); 
     
    
    
}
void imAlive(){
  CAN.sendMsgBuf(0x22, 0, 3, live);
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
   //  Serial.println(id);
   if(id==0x15){
      
    Speed_Pulse_Freq[0] = cdata[0];
    Speed_Pulse_Freq[1] = cdata[1];
    Speed_Pulse_DC = cdata[2];
    Speed_Pulse_Freq_value = (Speed_Pulse_Freq[1] << 8)|(Speed_Pulse_Freq[0]);

    Distance_Signal_Freq[0] = cdata[3];
    Distance_Signal_Freq[1] = cdata[4];
    Distance_Signal_DC = cdata[5];
    Distance_Signal_Freq_value = (Distance_Signal_Freq[1] << 8)|(Distance_Signal_Freq[0]);
    
   
    pwmDistanceFreq = int(Distance_Signal_Freq_value);
    pwmDistanceDC = float(Distance_Signal_DC);  
    
    pwmSpeedFreq = int(Speed_Pulse_Freq_value);
    pwmSpeedDC = float(Speed_Pulse_DC);

 
    if(pwmDistanceFreq && pwmDistanceDC)
      pwm_distance_signal_generate();
    else
      pwm_distance_signal_off();
    if(pwmSpeedFreq && pwmSpeedDC)
      pwm_speed_pulse_generate();
    else
      pwm_speed_pulse_off();
      
    
   }
  
}


void writeCan(){

  int freq_ch_1_A_B[2];
  int duty_ch_1_A_B[2];
  int freq_ch_2_A_B[2]; 
  int duty_ch_2_A_B[2];
  int freq_ch_3_A_B[2];
  int duty_ch_3_A_B[2]; 
  int freq_ch_4_A_B[2]; 
  int duty_ch_4_A_B[2]; 
  
  unsigned char Ch_1_A_B[8];
  unsigned char Ch_2_A_B[8];
  unsigned char Ch_3_A_B[8];
  unsigned char Ch_4_A_B[8];

  freq_ch_1_A_B[0] = int(freq[0]);
  freq_ch_1_A_B[1] = int(freq[1]);
  
  duty_ch_1_A_B[0] = int(duty[0]);
  duty_ch_1_A_B[1] = int(duty[1]);
  
  freq_ch_2_A_B[0] = int(freq[2]);
  freq_ch_2_A_B[1] = int(freq[3]);
  
  duty_ch_2_A_B[0] = int(duty[2]);
  duty_ch_2_A_B[1] = int(duty[3]);

  freq_ch_3_A_B[0] = int(freq[4]);
  freq_ch_3_A_B[1] = int(freq[5]);
  
  duty_ch_3_A_B[0] = int(duty[4]);
  duty_ch_3_A_B[1] = int(duty[5]);
  
  freq_ch_4_A_B[0] = int(freq[6]);
  freq_ch_4_A_B[1] = int(freq[7]);

  duty_ch_4_A_B[0] = int(duty[6]);
  duty_ch_4_A_B[1] = int(duty[7]);  

  
  for(int i = 0;i<2;i++){
    memcpy(&Ch_1_A_B[(4*i)], &freq_ch_1_A_B[i], 2);
    memcpy(&Ch_1_A_B[(4*i)+2], &duty_ch_1_A_B[i], 2);

    memcpy(&Ch_2_A_B[(4*i)], &freq_ch_2_A_B[i], 2);
    memcpy(&Ch_2_A_B[(4*i)+2], &duty_ch_2_A_B[i], 2);

    memcpy(&Ch_3_A_B[(4*i)], &freq_ch_3_A_B[i], 2);
    memcpy(&Ch_3_A_B[(4*i)+2], &duty_ch_3_A_B[i], 2);

    memcpy(&Ch_4_A_B[(4*i)], &freq_ch_4_A_B[i], 2);
    memcpy(&Ch_4_A_B[(4*i)+2], &duty_ch_4_A_B[i], 2);              
  }

   //Serial.println(Ch_1_A_B[0]);
  
  // Write PWM signals to CAN
     CAN.sendMsgBuf(0x18,0,8, Ch_1_A_B);
     delay(50);
     CAN.sendMsgBuf(0x19,0,8, Ch_2_A_B);
     delay(50);
     CAN.sendMsgBuf(0x1A,0,8, Ch_3_A_B);
     delay(50);
     CAN.sendMsgBuf(0x1B,0,8, Ch_4_A_B);    
     delay(50);
   
               
 
  
}


 void pwm_measure(){
   for(int i = 0; i< N_CHANNELS; i++){
      
      ontime[i] = pulseIn(pulse_ip[i],HIGH,TIMEOUT);        
      offtime[i] = pulseIn(pulse_ip[i],LOW,TIMEOUT);       
      if((ontime[i] != 0) && (offtime[i] != 0)){
         // Apply the moving average filter for ON TIME
          SUM_ON[i] = SUM_ON[i] - ON_READINGS[i][INDEX_ON[i]];       // Remove the oldest entry from the sum
          SUM_ON[i] = SUM_ON[i] + ontime[i];                 // Add the newest reading to the sum      
          ON_READINGS[i][INDEX_ON[i]] = ontime[i];           // Add the newest reading to the window     
          INDEX_ON[i] = (INDEX_ON[i]+1) % N_AVERAGE;   // Increment the index, and wrap to 0 if it exceeds the window size  
          AVERAGED_ON_TIME[i] = SUM_ON[i] / N_AVERAGE;      // Divide the sum of the window by the window size for the result
    
          // Apply the moving average filter for ON TIME
          SUM_OFF[i] = SUM_OFF[i] - OFF_READINGS[i][INDEX_OFF[i]];       // Remove the oldest entry from the sum
          SUM_OFF[i] = SUM_OFF[i] + offtime[i];                 // Add the newest reading to the sum      
          OFF_READINGS[i][INDEX_OFF[i]] = offtime[i];           // Add the newest reading to the window
          INDEX_OFF[i] = (INDEX_OFF[i]+1) % N_AVERAGE;   // Increment the index, and wrap to 0 if it exceeds the window size
          AVERAGED_OFF_TIME[i] = SUM_OFF[i] / N_AVERAGE;      // Divide the sum of the window by the window size for the result
         
          period[i] = AVERAGED_ON_TIME[i]+AVERAGED_OFF_TIME[i];
          freq[i] =  1000000/period[i]; // HZ
          duty[i] = (AVERAGED_ON_TIME[i]/period[i])*100;         
          //Serial.println(period[i]);
    } else {
          freq[i] = 0;
          duty[i] = 0;        
    }


  
}  
 }


void pwm_distance_signal_generate(){
 // Change frequency (example: 2kHz)
  //pwmDistanceFreq = FREQ_1;
  //pwmDistanceDC = DC_1;

// Set mode 14 (Fast PWM, ICR1 is TOP)
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);  
 // Set non-inverting mode for PWM output on OC1A (Pin 11 on Arduino MEGA)
  TCCR1A |= (1 << COM1A1);
// Start timer with prescaler 1
  TCCR1B |= (1 << CS10);  // Prescaler 1  
  
  uint16_t top1 = F_CPU / (1UL * pwmDistanceFreq) - 1;
  uint16_t duty1 = (pwmDistanceDC / 100.0) * (top1 + 1);
  ICR1 = top1;
  OCR1A = duty1;
}

void pwm_speed_pulse_generate(){
 // Change frequency (example: 2kHz)
  //pwmSpeedFreq = FREQ_2;
  //pwmSpeedDC = DC_2;
  TCCR4A |= (1 << WGM41);
  TCCR4B |= (1 << WGM42) | (1 << WGM43);  
 // Set non-inverting mode for PWM output on OC4A (Pin 6 on Arduino MEGA)
  TCCR4A |= (1 << COM4A1);
// Start timer with prescaler 1
  TCCR4B |= (1 << CS40);  // Prescaler 1  
    
  uint16_t top2 = F_CPU / (1UL * pwmSpeedFreq) - 1;
  uint16_t duty2 = (pwmSpeedDC / 100.0) * (top2 + 1);
  ICR4 = top2;
  OCR4A = duty2;
}

void pwm_distance_signal_off(){
  TCCR1A = 0;  // Clear register A configurations
  TCCR1B = 0;  // Clear register B configurations
  ICR1 = 0;
  OCR1A = 0;
}
void pwm_speed_pulse_off(){
  TCCR4A = 0;  // Clear register A configurations
  TCCR4B = 0;  // Clear register B configurations
  ICR4 = 0;
  OCR4A = 0;  
}
// END FILE
