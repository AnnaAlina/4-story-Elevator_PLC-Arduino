#include <Wire.h>
#include <Adafruit_MotorShield.h>

int currentFloor = 1;       // elevator starts on floor 1
int stepsPerFloor = 922;    // number of steps needed from the stepper motor to move 1 floor up or down
int doorDelay = 2000;       // time in mS for how long it takes for elevator doors to open or close
int doorOpenDelay = 2000;   // time in mS for how long the elevator doors should remain open
int inputs[] = {A0, A1, A2, A3}; // array of input pins
int outputs[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}; // array of output pins
// Array of input readings from PLC to determine which button was pressed
int buttons[] = {B0001, B0010, B0011, B0100, B0101, B0110, B1000, B1001, B1010, B1011};
int buttonCalls[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // array to keep track of button presses
bool input = false;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Pointer myMotor for stepper motor attached to Port 1 on motorshield
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);
// pointer motor1 for floor 1 DC motor Attached to M3 on motor shield
Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
// pointer motor2 for floor 2 DC motor attached to M4 on motor shield
Adafruit_DCMotor *motor2 = AFMS.getMotor(4);

void setup() {
  for (int i = 0; i < sizeof(inputs) / sizeof(int); i++){
    pinMode(inputs[i], INPUT); // declares all pins in input array as input pins 
  }
  for (int i = 0; i < sizeof(outputs) / sizeof(int); i++){
    pinMode(outputs[i], OUTPUT); // declares all pins in output arrays as output pins
  }
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->setSpeed(10);  // Stepper motor set to 10 rpm
  motor1->setSpeed(255);  // DC motor for floor 1 doors set to max speed
  motor2->setSpeed(255);  // DC motor for floor 2 doors set to max speed
  // Sets interrupt on Pin 2 to call newInput() whenever pin 2 changes from low to high
  attachInterrupt(digitalPinToInterrupt(2), newInput, RISING);
  closeDoor(4); // door 4 opens on startup so this closes the door
}

void newInput(){      // interrupt service routine to take in new input from PLC
  int buttonPressed;
  input = true;
  delayMicroseconds(5000); // delay to make sure all PLC relays have closed before reading input
  buttonPressed = PINC & B1111;   // takes inputs from A0-A3 as a 4-bit binary input
  for (int i = 0; i < 10; i++){
    if (buttonPressed == buttons[i]){   // compares input to array to determine button press
      buttonCalls[i] = 1;      // stores button input into button calls array
    }
  }
  buttonLights();  // updates button indicator lights                
}



void loop() {
  bool elevatorInUse = false;    // boolean flag for whether elevator has been called
  for (int i = 0; i <=2; i++){
    if (buttonCalls[2*i] || buttonCalls[i+6]){  //checks if up button or floor button were pressed
      elevatorInUse = true;
      changeFloor(i+1);          // moves elevator to next floor
      buttonCalls[2*i] = 0;      // clear button calls from queue once elevator arrives
      buttonCalls[i+6] = 0;      // clear button calls from queue once elevator arrives
      buttonLights();            // update button light indicators
      openDoor(currentFloor);    // opens elevator door on current floor
      delay(doorOpenDelay);
      closeDoor(currentFloor);   // closes elevator door on current floor
    }
  }
  for (int i = 2; i >= 0; i--){
    if (buttonCalls[2*i + 1] || buttonCalls[i+7]){  //checks if down button or floor button were pressed
      elevatorInUse = true;
      changeFloor(i+2);          // moves elevator to next floor
      buttonCalls[2*i+1] = 0;    // clear button calls from queue once elevator arrives
      buttonCalls[i+7] = 0;      // clear button calls from queue once elevator arrives
      buttonLights();            // update button light indicators
      openDoor(currentFloor);    // opens elevator door on current floor
      delay(doorOpenDelay);
      closeDoor(currentFloor);   // closes elevator door on current floor
    }
  }
  delay(doorDelay);
  if (!elevatorInUse){            // if no buttons have been pressed
    input = false;               
    changeFloor(1);               // change floor to floor 1
    openDoor(currentFloor);       // open elevator doors  
    Serial.println("No button presses"); // print command left in because it possibly fixed a bug
    while(1){                     // infinite loop until New button input
      Serial.println("Still here");  // // print command left in because it possibly fixed a bug
      if (input==true){                 // if new input received
        Serial.println("New button press"); // print command left in because it possibly fixed a bug

        closeDoor(currentFloor);  // close elevator doors
        break;
      }  
    }
  }   
}

void buttonLights(){
  int floorButtons[] = {4, 5, 6, 7, 8, 9}; // array of pins tied to indicator lights via optocoupler
  for (int i = 0; i <=5; i++){
    if (buttonCalls[i] == 1){ // checks whether any up or down buttons have been pressed
      digitalWrite(floorButtons[i], HIGH); // turns on indicator light
    }else{
      digitalWrite(floorButtons[i], LOW); // turns off indicator light
    }
  } 
}




void changeFloor(int newFloor){
  int floorDifference = abs(newFloor - currentFloor); // number of floors stepper motor runs for
  int upOrDown = (currentFloor < newFloor) ? BACKWARD : FORWARD; // decides stepper motor direction
  myMotor->step(stepsPerFloor * floorDifference, upOrDown, SINGLE); // runs stepper motor
  currentFloor = newFloor; // updates current floor
}

void openDoor(int floorDoor){ // opens elevator doors for each floor
  if (floorDoor == 1){
    motor1->run(FORWARD);     // starts floor 1 door motor
    delay(doorDelay);         // delay to give motor enough time to open/close door completely
    motor1->run(RELEASE);     // stops floor 1 door motor
  } else if (floorDoor == 2){
    motor2->run(FORWARD);     // starts floor 2 door motor
    delay(doorDelay);         // delay to give motor enough time to open/close door completely
    motor2->run(RELEASE);     // stops floor 2 door motor
  } else if(floorDoor == 3){
    digitalWrite(11, HIGH);   // starts floor 3 door motor
    delay(doorDelay);         // delay to give motor enough time to open/close door completely
    digitalWrite(11, LOW);    // stops floor 3 door motor
  } else{
    digitalWrite(13, HIGH);   // starts floor 4 door motor
    delay(doorDelay);         // delay to give motor enough time to open/close door completely
    digitalWrite(13, LOW);    // stops floor 4 door motor
  }
}

void closeDoor(int floorDoor){ // closes elevator doors for each floor
  if (floorDoor == 1){
    motor1->run(BACKWARD);     // starts floor 1 door motor
    delay(doorDelay);          // delay to give motor enough time to open/close door completely
    motor1->run(RELEASE);      // stops floor 1 door motor
  } else if (floorDoor == 2){
    motor2->run(BACKWARD);     // starts floor 2 door motor
    delay(doorDelay);          // delay to give motor enough time to open/close door completely
    motor2->run(RELEASE);      // stops floor 2 door motor
  } else if(floorDoor == 3){
    digitalWrite(10, HIGH);    // starts floor 3 door motor
    delay(doorDelay);          // delay to give motor enough time to open/close door completely
    digitalWrite(10, LOW);     // stops floor 3 door motor
  } else{
    digitalWrite(12, HIGH);    // starts floor 4 door motor
    delay(doorDelay);          // delay to give motor enough time to open/close door completely
    digitalWrite(12, LOW);     // stops floor 4 door motor
  }
}
