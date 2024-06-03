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


// SETTINGS - MOTOR
#define IN1 9  // unipolar motor pins (set all pins)
#define IN2 6  //motor driver step pin
#define IN3 8
#define IN4 7        //motor driver dir pin
#define motorType 1  //1 for unipolar, 2 for motor drive board (e.g. A4988)


// SETTINGS - HALL SENSOR



#if motorType == 1
AccelStepper stepper(4, IN1, IN3, IN2, IN4);  // Initialise UniPolar Motor
#elif motorType == 2
AccelStepper stepper(1, IN2, IN4);  //Initialise Motor Driver (e.g. A4988)
#endif



// Variable

bool Error = false;  // homing error flag
bool homingSensorFound = true;
int homingType = 4;
int sensorLimitMaxValue = 512;
int sensorLimitMinValue = 512;
bool runOnce = true;
int analogSensorThreshold = 430;  // set the threshold below which the homing sensor is triggered
bool sensorTypeIsDigtial = true;
bool activeHIGH;



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
  if (homingType == 0) {                                      //measure analog hall sensor voltage low trigger
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = analogRead(SENSOR);     // analog sensor
      if (stepper.distanceToGo() <= 0) {  // if we have moved to 3000 then raise error.
        return true;
      }

    } while (HallValue > analogSensorThreshold);  // analog sensor
  } else if (homingType == 1) {                   //measure analog Hall Sensor voltage high trigger
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = analogRead(SENSOR);     // analog sensor
      if (stepper.distanceToGo() <= 0) {  // if we have moved to 3000 then raise error.
        return true;
      }

    } while (HallValue < analogSensorThreshold);  // analog sensor
  } else if (homingType == 2) {                   //measure digital High Trigger
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = digitalRead(SENSOR);    // digital sensor
      if (stepper.distanceToGo() <= 0) {  // if we have moved to 3000 then raise error.
        return true;
      }
    } while (HallValue == LOW);  // active HIGH digital sensor
  } else if (homingType == 3) {  //measure digtial LOW Trigger
    do {
      stepper.run();                      // run the stepper one step at a time
      HallValue = digitalRead(SENSOR);    // digital sensor
      if (stepper.distanceToGo() <= 0) {  // if we have moved to 3000 then raise error.
        return true;
      }
    } while (HallValue == HIGH);     // active LOW digital sensor
  } else if (homingType == 4) {      //check is Hall Sensor is active HIGH or LOW
    HallValue = analogRead(SENSOR);  // analog sensor

    if (HallValue > 470 && HallValue < 530) {
      sensorTypeIsDigtial = false;
      Serial.println("sensor analog");
      
    }
    if (sensorTypeIsDigtial) {
      activeHIGH = HallValue < 300 ? true : false;
      Serial.println("sensor digital");
      return true;
    }

    do {
      stepper.run();                   // run the stepper one step at a time
      HallValue = analogRead(SENSOR);  // analog sensor

      if (HallValue > 529) {
        sensorLimitMaxValue = HallValue;
      }

      if (HallValue < 471) {
        sensorLimitMinValue = HallValue;
      }

    } while (stepper.distanceToGo() > 0);  // analog sensor
    if (sensorLimitMaxValue > 529) {
      activeHIGH = true;
    } else if (sensorLimitMinValue < 471) {
      activeHIGH = false;
    } else {
      homingSensorFound = false;
    }
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



  //Error = Locate_Home();  // home
}

void loop() {  // runs forever.

  if (runOnce) {
    homingType = 4;
    Error = Locate_Home();
    if (!Error && homingSensorFound) {
      if (activeHIGH) {
        analogSensorThreshold = sensorLimitMaxValue - 15;
        homingType = 1;
        Error = Locate_Home();
        if (!Error) {
          Serial.print("Sensor type is analog and active High with threshold of: ");
          Serial.println(analogSensorThreshold);
        } else {
          Serial.println("Home Sensor not found");
        }

      } else {
        analogSensorThreshold = sensorLimitMinValue + 15;
        homingType = 0;
        Error = Locate_Home();
        if (!Error) {
          Serial.print("Sensor type is analog and active low with threshold of: ");
          Serial.println(analogSensorThreshold);
        } else {
          Serial.println("Home Sensor not found");
        }
      }
    } else if (Error && homingSensorFound) {
      if (activeHIGH) {
        Error = false;
        homingType = 2;
        Error = Locate_Home();
        if (!Error) {
          Serial.println("Sensor type is Digital and Active High");
        } else {
          Serial.println("Home Sensor not found");
        }
      } else {
        Error = false;
        homingType = 3;
        Error = Locate_Home();
        if (!Error) {
          Serial.println("Sensor type is Digital and Active Low");
        } else {
          Serial.println("Home Sensor not found");
        }
      }
    } else {
      Serial.println("Home Sensor not found");
    }
  }
  runOnce = false;
}
