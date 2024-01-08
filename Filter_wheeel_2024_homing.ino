#include <AccelStepper.h>  //stepper library

// Constants
#define SENSOR A3      // Hall senser pin
#define hallPower A2   // hall 5V pin
#define hallCom A1     // hall GND pin
#define maxSpeed 400   // max motor speed
#define Speed 400      // current speed
#define setAccel 1000  // stepper accelaration


// SETTINGS - MAIN PARAMETERS
#define stepsPerRevolution 2048  //set number motor steps for wheel to rotate once
#define offSetResolution 5       //number of motor steps for one offset change
#define homingOffsetSteps 56     // number of steps from homing point to centre of first filter

// SETTINGS - MOTOR
#define IN1 9  // unipolar motor pins (set all pins)
#define IN2 6  //motor driver step pin
#define IN3 8
#define IN4 7        //motor driver dir pin
#define motorType 1  //1 for unipolar, 2 for motor drive board (e.g. A4988)


// SETTINGS - HALL SENSOR
#define analogSensorThreshold 430  // set the threshold below which the homing sensor is triggered
#define HALLSENSORTYPE 2           // SET to 1 for ANALOG HALL Sensor, CHANGE to 2 for DIGITAL HALL SENSOR
#define HALLACTIVETYPE 2           // SET to 1 for ACTIVE HIGH Digital Sensor, CHANGE to 2 for ACTIVE LOW digital Sensor

#if motorType == 1
AccelStepper stepper(4, IN1, IN3, IN2, IN4);  // Initialise UniPolar Motor
#elif motorType == 2
AccelStepper stepper(1, IN2, IN4);  //Initialise Motor Driver (e.g. A4988)
#endif



// Variable

bool Error = false;  // homing error flag
int homingType = 0;
int sensorLimitValue = 512;




void sendSerial(String message) {  // handle all serial messages to send
  Serial.println(message);
}




void motor_Off() {  // power down the stepper to save battery
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}



bool Locate_Home() {  // locate home
  int HallValue;
  stepper.runToNewPosition(stepper.currentPosition() - 500);  // back magnet off sensor
  stepper.moveTo(stepsPerRevolution * 1.2);                   // set move to as 120% of full revelution so if not homed by then raise error.
  if (homingType == 0) { //measure analog hall sensor voltage low trigger
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = analogRead(SENSOR);     // analog sensor
      if (stepper.distanceToGo() <= 0) {  // if we have moved to 3000 then raise error.
        return true;
      }

    } while (HallValue > analogSensorThreshold);  // analog sensor
  } else if (homingType == 1) {//measure analog Hall Sensor voltage high trigger
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = analogRead(SENSOR);     // analog sensor
      if (stepper.distanceToGo() <= 0) {  // if we have moved to 3000 then raise error.
        return true;
      }

    } while (HallValue < analogSensorThreshold);  // analog sensor
  } else if (homingType == 2) { //measure digital High Trigger
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = digitalRead(SENSOR);    // digital sensor
      if (stepper.distanceToGo() <= 0) {  // if we have moved to 3000 then raise error.
        return true;
      }
    } while (HallValue == LOW);  // active HIGH digital sensor
  } else if (homingType == 3) { //measure digtial LOW Trigger
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = digitalRead(SENSOR);    // digital sensor
      if (stepper.distanceToGo() <= 0) {  // if we have moved to 3000 then raise error.
        return true;
      }
    } while (HallValue == HIGH);  // active LOW digital sensor
  } else if (homingType == 4) { //check is Hall Sensor is N or S detecting
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = analogRead(SENSOR);     // analog sensor
      if (HallValue > 600) {
        if (HallValue > sensorLimitValue) {
          sensorLimitValue = HallValue;
        }
      } else if (HallValue < 430) {
        if (HallValue < sensorLimitValue) {
          sensorLimitValue = HallValue;
        }
      }

    } while (stepper.distanceToGo() > 0);  // analog sensor
  }
  stepper.stop();                 // stop stepper as we have homed.
  stepper.setCurrentPosition(0);  // set stepper position as 0 (home).
                                  // set filter position as 0 (i.e homed).
  return false;                   // return with no error.
}




void setup() {                 // runs once.
  Serial.begin(9600);          // start serial
  pinMode(hallCom, OUTPUT);    // hall sensor ground pin as output
  pinMode(hallPower, OUTPUT);  // hall sensor 5v pin as output
  pinMode(SENSOR, INPUT);      // hall sensor pin as input
  pinMode(IN1, OUTPUT);        // motor pins as outputs.
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stepper.setCurrentPosition(0);      // set stepper position = 0
  stepper.setMaxSpeed(maxSpeed);      // set max speer
  stepper.setSpeed(Speed);            // set speed
  stepper.setAcceleration(setAccel);  // set acceleration
  digitalWrite(hallCom, LOW);         // power hall sensor.
  digitalWrite(hallPower, HIGH);



  Error = Locate_Home();  // home
}

void loop() {  // runs forever.

  homingType = 4;
  Locate_Home();
  if (sensorLimitValue > 512) {
    analogSensorThreshold = 650;
    homingType = 1;
    Locate_Home();
  } else {
    analogSensorThreshold = 430;
    homingType = 0;
    Locate_Home();
  }
  if (Error) {
    Error = false;
    homingType = 2;
    Locate_Home();
  } 
  if (Error) {
    Error = false;
    homingType = 3;
    Locate_Home();
  }
  if (Error) {  // if homing does not work print error.
    Serial.println("Failed to find a homing sensor");
    Error = false;
  }
}
