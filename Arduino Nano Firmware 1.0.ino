#include <AccelStepper.h> //stepper library
#include <EEPROM.h>       //eprom library

// Constants
#define SENSOR A3    // Hall senser pin
#define hallPower A2 // hall 5V pin
#define hallCom A1   // hall GND pin
#define upButton 5    // UpButton Pin
#define downButton 4  // Down BUtton Pin
#define maxSpeed 400  // max motor speed
#define Speed 400     // current speed
#define setAccel 1000 // stepper accelaration
#define buzzerPin 2
#define ledPin 3

// SETTINGS - MAIN PARAMETERS
#define stepsPerRevolution 2048 //set number motor steps for wheel to rotate once
#define numberOfFilters 5
#define offSetResolution 5 //number of motor steps for one offset change
#define homingOffsetSteps 56 // number of steps from homing point to centre of first filter
#define buzzerConnected 0 //1 to enable on pin 2, 0 to disable
#define ledConnected 1 //1 to enable on pin 3, 0 to disable 

// SETTINGS - MOTOR
#define IN1 9 // unipolar motor pins (set all pins)
#define IN2 6 //motor driver step pin
#define IN3 8
#define IN4 7 //motor driver dir pin
#define motorType 1 //1 for unipolar, 2 for motor drive board (e.g. A4988)


// SETTINGS - HALL SENSOR
#define analogSensorThreshold 430 // set the threshold below which the homing sensor is triggered
#define HALLSENSORTYPE 2          // SET to 1 for ANALOG HALL Sensor, CHANGE to 2 for DIGITAL HALL SENSOR
#define HALLACTIVETYPE 2          // SET to 1 for ACTIVE HIGH Digital Sensor, CHANGE to 2 for ACTIVE LOW digital Sensor

#if motorType == 1
  AccelStepper stepper(4, IN1, IN3, IN2, IN4); // Initialise UniPolar Motor
#elif motorType == 2
  AccelStepper stepper(1, IN2, IN4); //Initialise Motor Driver (e.g. A4988)
#endif


const String I_Strings[] = {"Chemistorge_Filter_1.1", "FW3.1.5", "P", "S/N:001", "Max Speed " + String(maxSpeed), "Jitter 1", "PX Offset ", "Theshold 1", "FilterSlots " + String(numberOfFilters), "Pulse Width 4950uS"};

// Variable
int filterPos[numberOfFilters+1]; //Initiaise positions Array
int posOffset[numberOfFilters]; // Initialise Offset Array
bool Error = false;             // homing error flag
int currPos = 0;                // Start at 0
int upButtonState = 1;          // inital state of buttons (Active LOW so set HIGH.
int downButtonState = 1;        // inital state of buttons (Active LOW so set HIGH.

void serialFlush()
{ // functional to flush Serial in buffer if needed.
  if (Serial.available() > 0)
  {
    Serial.read();
  }
}

void writeIntIntoEEPROM(int address, int number)
{ // write one int16 to eprom
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}
int readIntFromEEPROM(int address)
{ // read int16 from eprom.
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}
void sendSerial(String message)
{ // handle all serial messages to send
  Serial.println(message);
}
void (*resetFunc)(void) = 0; // allow reset of arduino via code

void epromSave(bool showOutput)
{ // Save offset data to eprom, and optionally read back data.
  int Value;
  // EEPROM.write(address, value);
  for (int i = 0; i < numberOfFilters; i++)
  {
    writeIntIntoEEPROM(i * 2 + 50, posOffset[i]);
  }
  if (showOutput)
  {
    for (int i = 0; i < numberOfFilters; i++)
    {
      Value = readIntFromEEPROM(i * 2 + 50);
      sendSerial("P" + String(i + 1) + " Offset " + String(Value));
    }
  }
}

void motor_Off()
{ // power down the stepper to save battery
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

#if buzzerConnected == 1
  void playTone(int beeps) 
  { 
    for (int i = 0; i < beeps; i++) 
    { 
      tone(buzzerPin, 1000, 100); 
      delay(200); 
    } 
  }  
#endif

#if ledConnected == 1
  void playLed(int flashes)
  {
    for (int i = 0; i < flashes; i++)
    {
      digitalWrite(ledPin, HIGH);
      delay(300);
      digitalWrite(ledPin, LOW);
      delay(300);
    }
  }
#endif

bool Locate_Home()
{ // locate home
  int HallValue;
  stepper.runToNewPosition(stepper.currentPosition() - 500); // back magnet off sensor
  stepper.moveTo(stepsPerRevolution*1.2);  // set move to as 120% of full revelution so if not homed by then raise error.
  do
  {
    stepper.run(); // run the stepper one step at a time
#if HALLSENSORTYPE == 1
    HallValue = analogRead(SENSOR); // analog sensor
#elif HALLSENSORTYPE == 2
    HallValue = digitalRead(SENSOR); // digital sensor
#endif
    if (stepper.distanceToGo() <= 0)
    { // if we have moved to 3000 then raise error.
      return true;
    }
#if HALLSENSORTYPE == 1
  } while (HallValue > analogSensorThreshold); // analog sensor
#elif HALLSENSORTYPE == 2
  #if HALLACTIVETYPE == 1
    } while (HallValue == LOW); // active HIGH digital sensor
  #elif HALLACTIVETYPE == 2
    } while (HallValue == HIGH); // active LOW digital sensor
  #endif
#endif
  stepper.stop();                // stop stepper as we have homed.
  stepper.setCurrentPosition(0); // set stepper position as 0 (home).
  currPos = 0;                   // set filter position as 0 (i.e homed).
  return false;                  // return with no error.
}

void goToLocation(int newPos)
{ // got to new position.
  if ((newPos < 1) || (newPos > numberOfFilters))
  { // only allow is between 1 and number of filters
    return;
  }
  else if (newPos < currPos)
  { // if new postion is less that current, then home to prevent backlash errors.
    Error = Locate_Home();
  }
  stepper.runToNewPosition(filterPos[newPos] + posOffset[newPos - 1]); // run for from current position to new and include offsets.
  currPos = newPos;
  #if ledConnected == 1                                                   // update position
    playLed(currPos);
  #endif
  #if buzzerConnected == 1
    playTone(currPos);
  #endif
  motor_Off(); // turn off motor coil to save power.
}

void handleSerial(char firstChar, char secondChar)
{                                    // handle valid incoming serial messages.
  int number = int(secondChar - 48); // convert second char to number (ASCI code for 0 is 48 so 48 - 48 is 0, ASCI code for 1 is 49 so 49 - 48 is 1 etc.
  int tmpPoss;                       // hold current position during change.
  switch (toupper(firstChar))
  { // case insensitive //sorts messages by first letter.

  case 'B':            // move back XX places. (ASCOM uses this command for position)
    tmpPoss = currPos; // record current position
    for (int i = 0; i < number; i++)
    { // loop and -1 from current position.
      tmpPoss--;
      if (tmpPoss == 0)
      { // if we reach 0 then wrap to max filter number.
        tmpPoss = numberOfFilters;
      }
    }
    goToLocation(tmpPoss);             // update to new postion.
    sendSerial("P" + String(currPos)); // reply to serial with new position.
    break;

  case 'F':            // move forward XX places. (ASCOM uses this command for position)
    tmpPoss = currPos; // record current position
    for (int i = 0; i < number; i++)
    { // loop and +1 from current position.
      tmpPoss++;
      if (tmpPoss == numberOfFilters + 1)
      { // if we reach out of range then wrap to 1.
        tmpPoss = 1;
      }
    }
    goToLocation(tmpPoss);             // update to new postion.
    sendSerial("P" + String(currPos)); // reply to serial with new position.
    break;

  case 'G':                            // go to Position XX (INDI uses this command for position)
    goToLocation(number);              // update to new postion.
    sendSerial("P" + String(currPos)); // reply to serial with new position.
    break;

  case 'I': // Information
    if (!(number == 2) && !(number == 6))
    {                                // stock information for all expect I2 and I6
      sendSerial(I_Strings[number]); // reply with stock information from array.
    }
    else if (number == 2)
    { // I2 reply with current position eg. P2
      sendSerial(I_Strings[number] + String(currPos));
    }
    else if (number == 6)
    { // I2 reply with current offset for current position eg. P2 Offset 5
      sendSerial("P" + String(currPos) + " Offset " + String(posOffset[currPos - 1] / 5));
    }
    break;

  case 'M': // return a dummy pulse width value
  case 'N':
    sendSerial(I_Strings[9]); // variable not used so don't change
    break;

  case 'O': // return offset for position XX  eg. (O2) >> P2 Offset 5
    if (number > 0 && number < numberOfFilters+1)
    {
      sendSerial("P" + String(number) + " Offset " + String(posOffset[number - 1] / 5));
    }
    break;

  case 'R':
  { // reset commands.
    switch (number)
    {
    case 0: // R0/R1 reset arduino.
    case 1:
      resetFunc();
      break;

    case 2: // R2 reset all offsets to 0 and save to eprom
      for (int i = 0; i < numberOfFilters; i++)
      {
        posOffset[i] = 0;
      }
      epromSave(false);
      Locate_Home();
      goToLocation(1);
      sendSerial("Calibration Removed");
      break;

    case 3:                   // R3 Send dummy jitter value.
      sendSerial("Jitter 1"); // not used so dummy value
      break;

    case 4:                         // R4 send dummy motor speed
      sendSerial("Max Speed 100%"); // not used so dummy value
      break;

    case 5:                      // R5 send dummy threshold.
      sendSerial("Threshold 1"); // not used so dummy value
      break;

    case 6: // R6 Save offsets to eprom (they are anyway when changed)
      epromSave(false);
      break;
    }
  }

  break;

  case 'S':                   // Send dummy speed
    sendSerial("Speed=100%"); // variable not used so don't change
    break;

  case 'T': // send raw sensor value, not useful but needed for ASCOM driver.
    sendSerial(String(SENSOR));
    break;

  case '<':                       // Send dummy speed
    sendSerial("Backwards 100%"); // variable not used so don't change
    break;

  case '>':                     // Send dummy speed
    sendSerial("Forward 100%"); // variable not used so don't change
    break;

  case '[': // send dummy values
  case ']':
    sendSerial("Jitter 1"); // variable not used so don't change
    break;

  case '(': // (X increase offset for current position by 1 unit (1 unit = X steps)
    tmpPoss = currPos;
    posOffset[tmpPoss - 1] = posOffset[tmpPoss - 1] + offSetResolution;
    Locate_Home();
    goToLocation(tmpPoss);
    epromSave(false);
    sendSerial("P" + String(currPos) + " Offset " + String(posOffset[currPos - 1] / 5));
    break;

  case ')': //(X decrease offset for current position by 1 unit (1 unit = X steps)
    tmpPoss = currPos;
    posOffset[tmpPoss - 1] = posOffset[tmpPoss - 1] - offSetResolution;
    Locate_Home();
    goToLocation(tmpPoss);
    epromSave(false);
    sendSerial("P" + String(currPos) + " Offset " + String(posOffset[currPos - 1] / 5));
    break;

  case '{': // send dummy values.
  case '}':
    sendSerial("Theshold 1"); // variable not used so don't change
    break;

  default: // unknown command handled.
    Serial.println("Command Unknown");
  }
}
void setup()
{                             // runs once.
  Serial.begin(9600);         // start serial
  pinMode(hallCom, OUTPUT);   // hall sensor ground pin as output
  pinMode(hallPower, OUTPUT); // hall sensor 5v pin as output
  pinMode(SENSOR, INPUT);     // hall sensor pin as input
  pinMode(IN1, OUTPUT);       // motor pins as outputs.
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(upButton, INPUT_PULLUP); // button pins as input with pull up resister (active low).
  pinMode(downButton, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  stepper.setCurrentPosition(0);     // set stepper position = 0
  stepper.setMaxSpeed(maxSpeed);     // set max speer
  stepper.setSpeed(Speed);           // set speed
  stepper.setAcceleration(setAccel); // set acceleration
  digitalWrite(hallCom, LOW);        // power hall sensor.
  digitalWrite(hallPower, HIGH);
  filterPos[0] = 0;
  for (int i = 0; i < numberOfFilters; i++)
  { // setup filter pos and read position off sets from eprom.
    int calcPos = (i*(stepsPerRevolution/numberOfFilters)) + homingOffsetSteps;
    filterPos[i+1] = calcPos;
    //Serial.print("Filter Pos ");
    //Serial.print(i+1);
    //Serial.print(" : Value: ");
    //Serial.println(filterPos[i+1]);
    posOffset[i] = readIntFromEEPROM(i * 2 + 50);
  }
  Error = Locate_Home(); // home
  goToLocation(1);       // goto position 1
}

void loop()
{                                        // runs forever.
  upButtonState = digitalRead(upButton); // read button pins and save state
  downButtonState = digitalRead(downButton);

  if (upButtonState == LOW && currPos < numberOfFilters)
  { // if up button pressed and position is less than number of filters increment position
    goToLocation(currPos + 1);
    sendSerial("P" + String(currPos));
    delay(150); // debounce time
  }
  else if (downButtonState == LOW && currPos > 1)
  { // if down button pressed and position is more than 1 decrement position
    goToLocation(currPos - 1);
    sendSerial("P" + String(currPos));
    delay(150); // debounce time
  }

  if (Serial.available() > 0)
  {               // if at least one byte in serial in buffer.
    char one = 0; // initilse char variable.
    char two = 0;
    delay(10); // Wait long enough time for 2 bytes to enter buffer.
    if (!(Serial.peek() > 32))
    {                // get rid of LF and CR and spaces if present.
      Serial.read(); // read and remove from buffer.
    }
    else if (Serial.available() == 1)
    {                // Get rid of single char instructions.
      Serial.read(); // read and remove from buffer.
    }
    else
    {
      one = Serial.read(); // read first Byte
      two = Serial.read(); // read second Byte
      if (!(isDigit(one)) && isDigit(two))
      {                         // check for valid instruction mask e.g. letter - number)
        handleSerial(one, two); // if valid handle it.
      }
      else
      {
        serialFlush(); // else flush invalid. (fail safe)
      }
    }
  }

  if (Error)
  { // if homing does not work print error.
    Serial.println("Homing Failed");
    Error = false;
  }
}
