#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

Servo Servo1;
const int servoPin = 9;
const int endpoint_pin = 7;
int angle = 0;

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 5); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, 3, 6);
// AccelStepper stepper3(1, 4, 7);

MultiStepper steppersControl;  // Create instance of MultiStepper

long gotoposition[3]; // An array to store the target positions for each stepper motor
long command = 0;
String input_serial;
const int enable_pin = 8;
static char mesaj[12];
int message_pos = 0;

void setup() {

  pinMode(enable_pin, OUTPUT);
  pinMode(endpoint_pin, INPUT);
  digitalWrite(enable_pin, LOW);

  stepper1.setMaxSpeed(16000); // Set maximum speed value for the stepper
  stepper1.setAcceleration(8000);
  stepper1.setSpeed(16000);
  stepper2.setMaxSpeed(8500);
  stepper2.setAcceleration(6800);

  Serial.begin(115200,SERIAL_8N1);
  Servo1.attach(servoPin);
  char input_serial_char = 'N';

  while(input_serial_char != 'S'){
   if (Serial.available() > 0 ) 
      {
        input_serial_char = Serial.read();    
        Serial.println("START " + String(input_serial_char));
      }
  }

  /// run motor till the endpoint is reached
  while (digitalRead(endpoint_pin) == HIGH) {
    stepper2.setSpeed(-1000);
    stepper2.runSpeed(); // Run the stepper motor at maximum speed
  }

  stepper2.setSpeed(0);
  stepper2.stop(); 
  stepper2.setCurrentPosition(0);
  digitalWrite(enable_pin, HIGH);
  // stepper1.moveTo(-16000); // 3 steps back
  Serial.println("stopped");
}

void loop() {

  if (Serial.available() > 0 ) 
  {
    char input_serial_char = Serial.read();// input_serial = Serial.readStringUntil('\r\n');
    
    Serial.println("NEW_messag " + String(input_serial_char));

    if( int(input_serial_char) != 13 && input_serial_char != '\n')
    {
      mesaj[message_pos] = input_serial_char;
      if(message_pos == 0) command = 0;
      message_pos ++;
    }
    else
    {
      mesaj[message_pos] = '\0';
      Serial.println("mesaj");
      Serial.println(mesaj);
      for (int i=1 ; i< message_pos; i++)
        command = command*10 +( mesaj[i] - '0');
      message_pos = 0;
    }

    char motorId = mesaj[0];
    // Store the target positions in the "gotopostion" array
    if (motorId == 'A') { digitalWrite(enable_pin, 0); gotoposition[0] = command*100;  stepper1.moveTo(command*100);}  // 800 steps - full rotation with quater-step resolution  
    if (motorId == 'B') { digitalWrite(enable_pin, 0); gotoposition[1] = command*10;  stepper2.moveTo(command*10);}
    if (motorId == 'E') { digitalWrite(enable_pin, command); }
    if (motorId == 'S') { angle = command; command = 0; }
    // steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
    // steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position
  }
  if (endpoint_pin == 0 ) stepper1.stop();
  stepper1.run();
  stepper2.run();
  Servo1.write(angle);
}
