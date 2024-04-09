#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

Servo Servo1;
const int servoPin = 9;
const int endpoint_pin = 7;
int angle = 0;

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 5); // (Type of driver: with 2 pins, STEP, DIR)
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
  stepper2.setMaxSpeed(8500); //8500
  stepper2.setAcceleration(14500); //6700  5480

  Serial.begin(115200,SERIAL_8N1);
  Servo1.attach(servoPin);
  
  // set position 0 for servomotor
  Servo1.write(0);
  char input_serial_char = 'N';

  // wait for start signal 
  while(input_serial_char != 'S'){
   if (Serial.available() > 0 ) 
      {
        input_serial_char = Serial.read();    
        Serial.println("START " + String(input_serial_char));
      }
  }
  // run motor till the endpoint is reached
  while (digitalRead(endpoint_pin) == LOW) {
    stepper2.setSpeed(-500);
    stepper2.runSpeed(); // Run the stepper motor at set speed
  }

  stepper2.setSpeed(0);
  stepper2.stop(); 
  stepper2.setCurrentPosition(0);
  digitalWrite(enable_pin, HIGH);
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

    switch (motorId)
    {
    // control motor B with 4/1 microstepping
    case 'B':
      digitalWrite(enable_pin, 0);
      stepper2.stop(); 
      gotoposition[1] = command*10; // Store the target positions in the "gotopostion" array
      stepper2.moveTo(command*10);
      break;
    
    // control motor A with 16/1 microstepping
    case 'A':
      digitalWrite(enable_pin, 0); 
      gotoposition[0] = command*100;
      stepper1.moveTo(command*100);
      break;
    
    // set enable pin for steppers On/Off
    case 'E': 
      digitalWrite(enable_pin, command);
      break;

    // Move Servo motor 
    case 'S':
      angle = command; 
      command = 0;
      break;

    // homing sequence 
    case 'H':
      homing_sequence();
      break;
    
    default:
      break;
    }
  //**
  }
  stepper1.run();
  stepper2.run();
  Servo1.write(angle);
}

void homing_sequence()
{
  digitalWrite(enable_pin, LOW);
  while (digitalRead(endpoint_pin) == LOW) {
    stepper2.setSpeed(-500);
    stepper2.runSpeed(); // Run the stepper motor at set speed
  }
  stepper2.setSpeed(0);
  stepper2.stop(); 
  stepper2.setCurrentPosition(0);
  digitalWrite(enable_pin, HIGH);
}


//** 
    // if (motorId == 'A') { digitalWrite(enable_pin, 0); gotoposition[0] = command*100;  stepper1.moveTo(command*100);}  // 800 steps - full rotation with quater-step resolution  
    // if (motorId == 'B') { digitalWrite(enable_pin, 0); stepper2.stop(); gotoposition[1] = command*10;  stepper2.moveTo(command*10);}
    // if (motorId == 'E') { digitalWrite(enable_pin, command); }
    // if (motorId == 'S') { angle = command; command = 0; }
    // if (motorId == 'H') { homing_sequence(); }
    // steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
    // steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position