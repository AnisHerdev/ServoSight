#include <Servo.h>
Servo panServo, tiltServo;
const int panPin = 11;
const int tiltPin = 10;

void setup() {
  Serial.begin(9600);
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  panServo.write(90); // Center
  tiltServo.write(90); // Center
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      int pan = cmd.substring(0, comma).toInt();
      int tilt = cmd.substring(comma+1).toInt();
      pan = constrain(pan, 0, 180);
      tilt = constrain(tilt, 0, 180);
      panServo.write(pan);
      tiltServo.write(tilt);
    }
  }
}
