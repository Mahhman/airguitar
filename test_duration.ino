unsigned int distance_buffer[] = {16000,
16000, 16000,16000, 16000};


int distance_index = 0;
const int distance_length = 5;

// Pin used for ultra-sonic sensor
const int trigPin = 6;
const int echoPin = 7;
int duration, cm;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);

}
//find the lowest non-zero number, returen zero if
//all are zeros
int find_min(int array[]){
  int index = array[0];
  for(int i = 0; i < 5; i++) {
    if(array[i] != 0){
      if(index = 0){
        index = array[i];
      }
      else if(index < array[i]){
        index = array[i];
      }
    }
  }
  return index;
}

void loop() {
  // put your main code here, to run repeatedly:

 digitalWrite(trigPin, LOW);
 delayMicroseconds(2);
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(5);
 digitalWrite(trigPin, LOW);
 
 duration = pulseIn(echoPin, HIGH, 4000);
 
 distance_buffer[distance_index++ 
    % distance_length] = duration / 20;
    
 cm = find_min(distance_buffer);

 //print duration
 Serial.println(cm);
}
