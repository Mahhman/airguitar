// An array containing the waveform
// of a guitar sound

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>


unsigned char waveform[] =
{125, 148, 171, 194, 209, 230, 252, 255,
253, 244, 235, 223, 207, 184, 169, 167,
163, 158, 146, 131, 126, 129, 134, 127,
105, 80, 58, 51,38, 22, 12,  2, 10, 35,
58, 75, 89, 103, 120, 141, 150, 148, 145,
144, 140, 129, 116, 105, 95, 86, 75, 72,
73, 76, 88, 103, 117, 121, 120, 115, 120,
143, 159, 162, 156, 155, 163, 184, 202,
214, 215, 211, 213, 212, 205, 196, 182,
162, 142, 118, 99, 84, 68, 54, 40, 28,
19, 10,  7,  0,  0,  5,  9, 14, 21, 33,
49, 59, 65, 75, 92, 110};

// We use this waveform to change the
// volume of the output
unsigned char waveformVolume[] =
{125, 148, 171, 194, 209, 230, 252, 255,
253, 244, 235, 223, 207, 184, 169, 167,
163, 158, 146, 131, 126, 129, 134, 127,
105, 80, 58, 51,38, 22, 12,  2, 10, 35,
58, 75, 89, 103, 120, 141, 150, 148, 145,
144, 140, 129, 116, 105, 95, 86, 75, 72,
73, 76, 88, 103, 117, 121, 120, 115, 120,
143, 159, 162, 156, 155, 163, 184, 202,
214, 215, 211, 213, 212, 205, 196, 182,
162, 142, 118, 99, 84, 68, 54, 40, 28,
19, 10,  7,  0,  0,  5,  9, 14, 21, 33,
49, 59, 65, 75, 92, 110};

// An array used as a buffer to avoid
// erroneous punctual distance
// measurements
unsigned int distance_buffer[] = {16000,
16000, 16000,16000, 16000};

const int distance_length = 5;
int distance_index = 0;

// The overflow values for 2 octaves

//min 62
//16
int frequencies[] = 
{99,106,113,120,127,
133,142,148,160,169,
177,188,199,212,223,
237};
int max_frequencies[]=
{ 78,79,82,88,94,99};//dummy 78


// Initial pitch
int pitch = 160;

// Initial volume and acceleration
// parameter
float volume = 0;

//audio playback on pin 3
byte speakerLeftPin = 2; 
byte speakerRightPin = 4; 

//index variable for position in
//waveform
volatile byte waveindex = 0;
volatile byte currentvalue = 0;

// Pin used for ultra-sonic sensor
const int trigPin = 6;
const int echoPin = 7;

// Pins for the potentiometers
const int sustainPin = A1;
const int sensitivityPin = A2;

// Pins for each finger of the left
// hand
const int finger1 = 8;
const int finger2 = 10;
const int finger3 = 11;
const int finger4 = 12;

int fingerValue = 0;

long duration, inches, cm;


// Added Global Variables
int sustainRead, sensitivityRead;


Adafruit_LIS3DH lis = Adafruit_LIS3DH();
int16_t acc_x = 0;
int16_t acc_y = 0;
int16_t acc_z = 0;
int16_t acc_x_last = 0;
int16_t acc_y_last = 0;
int16_t acc_z_last = 0;

void setup() {

  Serial.begin(9600);

  lis.begin(0x18);
  lis.setRange(LIS3DH_RANGE_4_G);  
  
  pinMode(speakerLeftPin,OUTPUT); //Speaker on pin 3
  pinMode(speakerRightPin,OUTPUT); //Speaker on pin 4

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(finger1,INPUT);
  pinMode(finger2,INPUT);
  pinMode(finger3,INPUT);
  pinMode(finger4,INPUT);
  
  
  /**************************
      PWM audio configuration
  ****************************/
  //set Timer2 to fast PWM mode
  //(doubles PWM frequency)
  bitSet(TCCR1A, WGM11); 
  bitSet(TCCR1B, CS10);
  bitClear(TCCR1B, CS11);
  bitClear(TCCR1B, CS12);
  
  //enable interrupts now that registers
  // have been set
  sei();
  
  
  /*************************
  Timer 1 interrupt configuration
  *************************/
  //disable interrupts while
  // registers are configured
  cli();
  
  /* Normal port operation, pins disconnected
  from timer operation (breaking pwm) */
  bitClear(TCCR1A, COM1A1);
  bitClear(TCCR1A, COM1A1);
  bitClear(TCCR1A, COM1A1);
  bitClear(TCCR1A, COM1A1);
  
  /* Mode 4, CTC with TOP set by register
  OCR1A. Allows us to set variable timing for
  the interrupt by writing new values to
  OCR1A. */
  bitClear(TCCR1A, WGM10);
  bitClear(TCCR1A, WGM11);
  bitSet(TCCR1B, WGM12);
  bitClear(TCCR1B, WGM13);
  
  /* set the clock prescaler to /8.  */
  bitClear(TCCR1B, CS10);
  bitSet(TCCR1B, CS11);
  bitClear(TCCR1B, CS12);
  
  /* Disable Force Output Compare for
  Channels A and B. */
  bitClear(TCCR1C, FOC1A);
  bitClear(TCCR1C, FOC1B);
  
  /* Initializes Output Compare
  Register A at 160 to set the
  initial pitch */
  OCR1A = 160;
  
  //disable input capture interrupt
  bitClear(TIMSK1, ICIE1); 
  //disable Output
  //Compare B Match Interrupt
  bitClear(TIMSK1, OCIE1B); 
  //enable Output
  //Compare A Match Interrupt
  bitSet(TIMSK1, OCIE1A); 
  //disable Overflow Interrupt
  bitClear(TIMSK1, TOIE1); 
  
  // enable interrupts now that
  // registers have been set
  sei();
}

// Timer overflow handler
ISR(TIMER1_COMPA_vect) {

 /* timer1 ISR.  Every time it
 is called it sets speakerpin to the
 next value in waveform[]. Frequency
 modulation is done by changing the
 timing between successive calls of
 this  function, e.g. for a 1KHz tone,
 set the  timing so that it runs
 through waveform[] 1000 times
 a second. */

 // reset waveindex if it has reached
 // the end of the array 

 if (waveindex > 101) { 
  waveindex = 0;
 }

 //Set the output value
 if (volume > 0.03) {
  analogWrite(speakerLeftPin,
   waveformVolume[waveindex]);

   analogWrite(speakerRightPin,
   waveformVolume[waveindex]);
   waveindex++;
 }
 else {
   analogWrite(speakerLeftPin, 0);
   analogWrite(speakerRightPin, 0);  
 }

 // Update the pitch
 OCR1A = pitch; 
}  


//find the lowest non-zero number, returen zero if
//all are zeros
int find_min(int array[]){
  int index = array[0];
  for(int i = 1; i < 5; i++) {
    if(array[i] > 0){
      if(index == 0){
        index = array[i];
      }
      else if(index > array[i]){
        index = array[i];
      }
    }
  }
  return index;
}



void determineParameters() {


 // convert the time into a distance
 // in centimetres
 // and store in buffer
 distance_buffer[distance_index++ 
  % distance_length] = duration / 20;

 //Find in the buffer the shortest
 // distance measured
 cm = 16000;
 for(int i = 0; i < distance_length; i++) {
  cm = find_min(distance_buffer);
 }


 
 float sustain = 
    map(sustainRead, 0,
    1024, 101, 130) / 100.0;
 int sensitivity =
    map(sensitivityRead,
    0, 1024, 100, 200);

 // Update the volume
 volume = volume / sustain;
 if (volume < 0) {
  volume = 0;
 }
 

  int16_t acc_x_diff = acc_x_last - acc_x;
  if( acc_x_diff < 0 ) { acc_x_diff = -acc_x_diff; }
  int16_t acc_y_diff = acc_y_last - acc_y;
  if( acc_y_diff < 0 ) { acc_y_diff = -acc_y_diff; }
  int16_t acc_z_diff = acc_z_last - acc_z;
  if( acc_z_diff < 0 ) { acc_z_diff = -acc_z_diff; }

  acc_x_last = acc_x;
  acc_y_last = acc_y;
  acc_z_last = acc_z;

  int16_t total_diff = acc_x_diff + acc_y_diff + acc_z_diff;


 // Update the volume value
 if (total_diff > 5 * (200 - sensitivity)) {
  volume += float( total_diff/100 )/1000;
  // pow(total_diff,
      // sensitivity / 100.0) / 50000;
 }

 // Check that volume is not higher than 1
 if(volume > .95) {
  volume = .95;
 }



 // Update the volume in the waveform
 for(int i = 0; i < 102; i++) {
  waveformVolume[i] =
   ((waveform[i] ) * volume);
 }



 // Set the pitch according to the distance
 // between the two hands and the
 // fingers pressed


// TESTING
 // pitch=70;
  long num;
 
 
 if(cm < 102 && cm > 0) {
  if(cm > 30) {
     num = ((cm - 30) / 24) * 4 + fingerValue - 1;
     if(num < 16) { pitch = frequencies[num]; }
     else {pitch = frequencies[15]; }
  }else{
   pitch = max_frequencies[fingerValue];
  }
 }else{
  num = 11 + fingerValue;
  if(num < 16) { pitch = frequencies[num]; }
  else {pitch = frequencies[15]; }
 }


  Serial.println(pitch);
 

 // Delay to avoid bouncing signals
 delay(50);
 
}


void loop() {  
 // Desactivate interputs, send a ping
 // message and wait for the answer.
 cli();
 digitalWrite(trigPin, LOW);
 delayMicroseconds(2);
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(5);
 digitalWrite(trigPin, LOW);
 
 duration = pulseIn(echoPin, HIGH, 4000);
 sei();

 // Check which fingers are pressed
 fingerValue = 5;
 if(!digitalRead(finger4)){
  fingerValue = 1;
 }
 if(!digitalRead(finger3)){
  fingerValue = 2;
 }
 if(!digitalRead(finger2)){
  fingerValue = 3;
 }
 if(!digitalRead(finger1)){
  fingerValue = 4;
 }

 // Update the sustain and
 // sensitivity values

 sustainRead = analogRead(sustainPin);
 sensitivityRead = analogRead(sensitivityPin);


  lis.read();
  acc_x = lis.x;
  acc_y = lis.y;
  acc_z = lis.z;

 
 determineParameters();
}
