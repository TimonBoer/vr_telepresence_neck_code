#include <Servo.h>

Servo left;
Servo right;
Servo front;
Servo back;
Servo yaw;
int current;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  left.attach(5);
  right.attach(9);
  front.attach(3);
  back.attach(6);
  yaw.attach(10);

  // left.write(90);
  // right.write(90);
  // front.write(90);
  // back.write(90);
  yaw.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()>0) {
    int inp = Serial.read();
    if (inp == 181){
      current = 0;
    } else {
      if (current == 0){
        left.write(inp);
      } else if (current == 1){
        right.write(inp);
      } else if (current == 2) {
        front.write(inp);
      } else if (current == 3) {
        back.write(inp);
      } else if (current == 4) {
        yaw.write(inp);
      }
      current += 1;
    }
  }
}
