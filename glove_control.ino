#define THUMB_PIN 34
#define INDEX_PIN 35
#define MID_PIN 32
#define RING_PIN 33
#define PINKY_PIN 25

int fingerValue[] = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600);
  // analogReadResolution(12);
}

void loop() {
  // thumbValue = analogRead(flexPin1);
  // indexValue = analogRead(flexPin2);

  // Serial.print("Thumb:");
  // Serial.print(thumbValue);
  // Serial.print(",");
  // Serial.print("Index:");
  // Serial.println(indexValue);

  // get all of our variables of interest
  float t = millis()/1000.0;
  fingerValue[0] = analogRead(THUMB_PIN);
  fingerValue[1] = analogRead(INDEX_PIN);
  fingerValue[2] = analogRead(MID_PIN);
  fingerValue[3] = analogRead(RING_PIN);
  fingerValue[4] = analogRead(PINKY_PIN);

  // write them all to console with tabs in between them
  Serial.print(t);         
  Serial.print("\t");
  Serial.print(fingerValue[0]);   
  Serial.print("\t");      
  Serial.print(fingerValue[1]); 
  Serial.print("\t");      
  Serial.print(fingerValue[2]); 
  Serial.print("\t");      
  Serial.print(fingerValue[3]); 
  Serial.print("\t");      
  Serial.println(fingerValue[4]); 
  
  // For example, at 2.5 seconds, this prints out like so, where \t
  // is the tab character, and \n is the newline character
  // 2500\t0.598\t-0.801\t\n

}