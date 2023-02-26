#include <Servo.h>

Servo myservo;
int speed1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.writeMicroseconds(1500); //set initial servo position if desired
  myservo.attach(9);  //the pin for the servo control 
  Serial.println("servo-test-22-dual-input");
}

void loop() {
  delay(1000);
for(int i =1000;i<2600;i+=200){
    myservo.write(i);
    Serial.println(i);
    delay(500);
  }
}
