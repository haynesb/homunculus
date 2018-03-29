//******************************************************************************************************************************************************************************************************
// MECCONTROL FIRMWARE
// Version: Arduino/Genuino Uno 1.3.0
//
// Copyright (C) 2015-2017 Tim Surtell
// www.surtell.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************************************************************************************************************************************

// Pin setup
enum PinSetup : byte {
  enuPinSetupUnused,
  enuPinSetupInput,
  enuPinSetupInputLatchingHigh,
  enuPinSetupInputLatchingLow,
  enuPinSetupOutput,
  enuPinSetupOutputPWM,
  enuPinSetupMeccanoid
};

// Lowest pins
const byte bytLowestDigitalPin = 2;
const byte bytLowestAnalogPin = 0;

// For Arduino/Genuino Uno boards
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  // Model and version
  #define strModelAndVersion "MECControl Arduino/Genuino Uno 1.3.0"
  
  // Highest pins
  const byte bytHighestDigitalPin = 13;
  const byte bytHighestAnalogPin = 5;
#endif

// For Arduino/Genuino Mega boards
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // Model and version
  #define strModelAndVersion "MECControl Arduino/Genuino Mega 1.3.0"
  
  // Highest pins
  const byte bytHighestDigitalPin = 53;
  const byte bytHighestAnalogPin = 15;
#endif

PinSetup arrDigitalPinSetup[bytHighestDigitalPin + 1];
PinSetup arrAnalogPinSetup[bytHighestAnalogPin + 1];

// Latch
const unsigned int intLatchDebounceInterval = 500;

bool arrDigitalLatchTriggered[bytHighestDigitalPin + 1];
unsigned long arrDigitalLatchDebounceStartTime[bytHighestDigitalPin + 1];
bool arrAnalogLatchTriggered[bytHighestAnalogPin + 1];
unsigned long arrAnalogLatchDebounceStartTime[bytHighestAnalogPin + 1];

// State
byte arrCurrentValue[bytHighestDigitalPin + 1][4];
byte arrStartValue[bytHighestDigitalPin + 1][4];
byte arrEndValue[bytHighestDigitalPin + 1][4];

unsigned int arrDuration[bytHighestDigitalPin + 1][4];
unsigned long arrStartTime[bytHighestDigitalPin + 1][4];

// Meccanoid
byte arrMeccanoidData[bytHighestDigitalPin + 1][4];
byte arrMeccanoidType[bytHighestDigitalPin + 1][4];

// Other
enum CommandResponse : byte {
  enuCommandResponseUnknown,
  enuCommandResponseOK,
  enuCommandResponseNone
};

String strCommand;
String strCommandPart[6];

CommandResponse bytCommandResponse;
bool booAcknowledge;

//******************************************************************************************************************************************************************************************************
// SETUP
//******************************************************************************************************************************************************************************************************

void setup() {
  // Initialize serial port
  Serial.begin(57600);

  // Wait for serial port to be ready
  while (!Serial) { }

  // Reset digital pins
  for (byte bytPin = bytLowestDigitalPin; bytPin <= bytHighestDigitalPin; bytPin++) {
    // Pins default to unused inputs
    arrDigitalPinSetup[bytPin] = enuPinSetupUnused;
    pinMode(bytPin, INPUT);

    // Reset latch
    arrDigitalLatchTriggered[bytPin] = false;
    arrDigitalLatchDebounceStartTime[bytPin] = 0;

    // Reset state
    for (byte bytPosition = 0; bytPosition <= 3; bytPosition++) {
      arrCurrentValue[bytPin][bytPosition] = 0;
      arrStartValue[bytPin][bytPosition] = 0;
      arrEndValue[bytPin][bytPosition] = 0;
      arrDuration[bytPin][bytPosition] = 0;
      arrStartTime[bytPin][bytPosition] = 0;
    }
  }

  // Reset analogue pins
  for (byte bytPin = bytLowestAnalogPin; bytPin <= bytHighestAnalogPin; bytPin++) {
    // Pins default to unused inputs
    arrAnalogPinSetup[bytPin] = enuPinSetupInput;

    // Reset latch
    arrAnalogLatchTriggered[bytPin] = false;
    arrAnalogLatchDebounceStartTime[bytPin] = 0;
  }

  // Acknowledge commands by default
  booAcknowledge = true;
}

//******************************************************************************************************************************************************************************************************
// LOOP
//******************************************************************************************************************************************************************************************************

void loop() {
  // Iterate through digital pins
  for (byte bytPin = bytLowestDigitalPin; bytPin <= bytHighestDigitalPin; bytPin++) {
    switch (arrDigitalPinSetup[bytPin]) {
      case enuPinSetupInputLatchingHigh:
        {
          // Check input for high latch trigger
          if (digitalRead(bytPin) && millis() - arrDigitalLatchDebounceStartTime[bytPin] > intLatchDebounceInterval) {
            arrDigitalLatchTriggered[bytPin] = true;
            arrDigitalLatchDebounceStartTime[bytPin] = millis();
          }
          break;
        }
      case enuPinSetupInputLatchingLow:
        {
          // Check input for low latch trigger
          if (!digitalRead(bytPin) && millis() - arrDigitalLatchDebounceStartTime[bytPin] > intLatchDebounceInterval) {
            arrDigitalLatchTriggered[bytPin] = true;
            arrDigitalLatchDebounceStartTime[bytPin] = millis();
          }
          break;
        }
      case enuPinSetupOutputPWM:
        {
          // Change PWM values

          // Does the current value equal the end value?
          if (arrCurrentValue[bytPin][0] != arrEndValue[bytPin][0]) {
            // Calculate the new value
            if ((millis() - arrStartTime[bytPin][0]) >= arrDuration[bytPin][0]) {
              arrCurrentValue[bytPin][0] = arrEndValue[bytPin][0];
            } else {
              arrCurrentValue[bytPin][0] = arrStartValue[bytPin][0] + (((float)(millis() - arrStartTime[bytPin][0]) / arrDuration[bytPin][0]) * (arrEndValue[bytPin][0] - arrStartValue[bytPin][0]));
            }

            // Set the PWM output
            analogWrite(bytPin, arrCurrentValue[bytPin][0]);
          }
          break;
        }
      case enuPinSetupMeccanoid:
        {
          // Change servo values
          byte bytValue;
          bool booUpdate = false;

          for (byte bytPosition = 0; bytPosition < 4; bytPosition++) {
            if (arrMeccanoidType[bytPin][bytPosition] == 1) {
              // Does the current value equal the end value?
              if (arrCurrentValue[bytPin][bytPosition] != arrEndValue[bytPin][bytPosition]) {
                // Calculate the new value
                if ((millis() - arrStartTime[bytPin][bytPosition]) >= arrDuration[bytPin][bytPosition]) {
                  bytValue = arrEndValue[bytPin][bytPosition];
                } else {
                  bytValue = arrStartValue[bytPin][bytPosition] + (((float)(millis() - arrStartTime[bytPin][bytPosition]) / arrDuration[bytPin][bytPosition]) * (arrEndValue[bytPin][bytPosition] - arrStartValue[bytPin][bytPosition]));
                }

                // Set the servo value, if there has been a change
                if (bytValue != arrCurrentValue[bytPin][bytPosition]) {
                  arrCurrentValue[bytPin][bytPosition] = bytValue;
                  arrMeccanoidData[bytPin][bytPosition] = bytValue;
                  booUpdate = true;
                }
              }
            }
          }

          // Only update if there has been a change to at least one servo value
          if (booUpdate) {
            meccanoidCommunicate(bytPin, 0);
          }

          break;
        }
      default:
        {
          // Do nothing
        }
    }

    // Iterate through analogue pins
    for (byte bytPin = bytLowestAnalogPin; bytPin <= bytHighestAnalogPin; bytPin++) {
      switch (arrAnalogPinSetup[bytPin]) {
        case enuPinSetupInputLatchingHigh:
          {
            // Check input for high latch trigger
            if (analogRead(bytPin) > 255 && millis() - arrAnalogLatchDebounceStartTime[bytPin] > intLatchDebounceInterval) {
              arrAnalogLatchTriggered[bytPin] = true;
              arrAnalogLatchDebounceStartTime[bytPin] = millis();
            }
            break;
          }
        case enuPinSetupInputLatchingLow:
          {
            // Check input for low latch trigger
            if (analogRead(bytPin) < 256 && millis() - arrAnalogLatchDebounceStartTime[bytPin] > intLatchDebounceInterval) {
              arrAnalogLatchTriggered[bytPin] = true;
              arrAnalogLatchDebounceStartTime[bytPin] = millis();
            }
            break;
          }
        default:
          {
            // Do nothing
          }
      }
    }
  }

  // See if there is a command waiting
  if (Serial.available()) {
    // Read in the command
    strCommand = Serial.readStringUntil('\n');
    strCommand.toUpperCase();

    // Get the first part of the command
    strCommandPart[0] = getWord(strCommand, 0);

    // Interpret the command - default response is Unknown
    bytCommandResponse = enuCommandResponseUnknown;

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // MeccanoidSetServoMotorPosition
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if (strCommandPart[0] == F("MNSSMP") || strCommandPart[0] == F("MECCANOIDSETSERVOMOTORPOSITION")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);
      strCommandPart[3] = getWord(strCommand, 3);
      strCommandPart[4] = getWord(strCommand, 4);

      byte bytPosition = strCommandPart[3].toInt();

      // Ensure the position values are within end stop range
      if (bytPosition < 24) {
        bytPosition = 24;
      }
      if (bytPosition > 232) {
        bytPosition = 232;
      }

      // Get the servo position if it currently disabled
      if (arrMeccanoidData[strCommandPart[1].toInt()][strCommandPart[2].toInt()] == 0xFA) {
        arrCurrentValue[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = meccanoidGetServoMotorPosition(strCommandPart[1].toInt(), strCommandPart[2].toInt());
      }

      // Store the start value, end value, start time and duration
      arrStartValue[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = arrCurrentValue[strCommandPart[1].toInt()][strCommandPart[2].toInt()];
      arrEndValue[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = bytPosition;
      arrStartTime[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = millis();
      arrDuration[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = strCommandPart[4].toInt();

      bytCommandResponse = enuCommandResponseOK;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // MeccanoidSetServoLEDColour
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("MNSSLC") || strCommandPart[0] == F("MECCANOIDSETSERVOLEDCOLOUR") || strCommandPart[0] == F("MECCANOIDSETSERVOLEDCOLOR")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);
      strCommandPart[3] = getWord(strCommand, 3);

      byte bytColour = strCommandPart[3].toInt();;

      // Ensure the colour values are within range
      if (bytColour > 7) {
        bytColour = 7;
      }

      // Set the colour
      arrMeccanoidData[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = bytColour + 240;
      meccanoidCommunicate(strCommandPart[1].toInt(), strCommandPart[2].toInt());

      bytCommandResponse = enuCommandResponseOK;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // MeccanoidSetLEDColour
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("MNSLC") || strCommandPart[0] == F("MECCANOIDSETLEDCOLOUR") || strCommandPart[0] == F("MECCANOIDSETLEDCOLOR")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);
      strCommandPart[3] = getWord(strCommand, 3);
      strCommandPart[4] = getWord(strCommand, 4);
      strCommandPart[5] = getWord(strCommand, 5);

      byte bytLED1 = 0;
      byte bytLED2 = 0;
      byte bytLEDPosition = 0;

      // Find the position of the LED
      for (byte bytPin = 0; bytPin < 4; bytPin++) {
        if (arrMeccanoidType[strCommandPart[1].toInt()][bytPin] == 2) {
          bytLEDPosition = bytPin;
        }
      }

      // Combine Red, Green, Blue and Fade values into two bytes: bytLED1 = 0GGGRRR; bytLED2 = 1FFFBBB
      bytLED1 =  0x3F & (((strCommandPart[3].toInt() << 3) & 0x38) | (strCommandPart[2].toInt() & 0x07));
      bytLED2 =  0x40 | (((strCommandPart[5].toInt() << 3) & 0x38) | (strCommandPart[4].toInt() & 0x07));

      // Set the colour
      arrMeccanoidData[strCommandPart[1].toInt()][bytLEDPosition] = bytLED1;
      meccanoidCommunicate(strCommandPart[1].toInt(), 0);
      arrMeccanoidData[strCommandPart[1].toInt()][bytLEDPosition] = bytLED2;
      meccanoidCommunicate(strCommandPart[1].toInt(), 0);

      bytCommandResponse = enuCommandResponseOK;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // MeccanoidGetServoMotorPosition
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("MNGSMP") || strCommandPart[0] == F("MECCANOIDGETSERVOMOTORPOSITION")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);

      // Return the servo position
      arrCurrentValue[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = meccanoidGetServoMotorPosition(strCommandPart[1].toInt(), strCommandPart[2].toInt());
      arrEndValue[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = arrCurrentValue[strCommandPart[1].toInt()][strCommandPart[2].toInt()];

      Serial.println(arrCurrentValue[strCommandPart[1].toInt()][strCommandPart[2].toInt()]);

      bytCommandResponse = enuCommandResponseNone;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // MeccanoidDisableServoMotor
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("MNDSM") || strCommandPart[0] == F("MECCANOIDDISABLESERVOMOTOR")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);

      // Disable the servo motor
      arrMeccanoidData[strCommandPart[1].toInt()][strCommandPart[2].toInt()] = 0xFA;
      meccanoidCommunicate(strCommandPart[1].toInt(), strCommandPart[2].toInt());

      bytCommandResponse = enuCommandResponseOK;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // StepperMotorStep
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("SMS") || strCommandPart[0] == F("STEPPERMOTORSTEP")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);
      strCommandPart[3] = getWord(strCommand, 3);
      strCommandPart[4] = getWord(strCommand, 4);
      strCommandPart[5] = getWord(strCommand, 5);
      strCommandPart[6] = getWord(strCommand, 6);

      // Determine step direction, number of steps and step interval
      int intNumberOfSteps = strCommandPart[5].toInt();
      bool booStepDirectionClockwise = (intNumberOfSteps > 0);
      intNumberOfSteps = abs(intNumberOfSteps);

      for (int intStep = 0; intStep < intNumberOfSteps; intStep++) {
        // Determine the next step in the pattern
        if (booStepDirectionClockwise) {
          switch (arrEndValue[strCommandPart[1].toInt()][0]) {
            case B00001100:
              arrEndValue[strCommandPart[1].toInt()][0] = B00000110;
              break;
            case B00000110:
              arrEndValue[strCommandPart[1].toInt()][0] = B00000011;
              break;
            case B00000011:
              arrEndValue[strCommandPart[1].toInt()][0] = B00001001;
              break;
            default:
              arrEndValue[strCommandPart[1].toInt()][0] = B00001100;
          }
        } else {
          switch (arrEndValue[strCommandPart[1].toInt()][0]) {
            case B00001100:
              arrEndValue[strCommandPart[1].toInt()][0] = B00001001;
              break;
            case B00001001:
              arrEndValue[strCommandPart[1].toInt()][0] = B00000011;
              break;
            case B00000011:
              arrEndValue[strCommandPart[1].toInt()][0] = B00000110;
              break;
            default:
              arrEndValue[strCommandPart[1].toInt()][0] = B00001100;
          }
        }

        // Set the outputs to high or low depending on their position in the pattern
        digitalWrite(strCommandPart[1].toInt(), arrEndValue[strCommandPart[1].toInt()][0] & B00001000);
        digitalWrite(strCommandPart[2].toInt(), arrEndValue[strCommandPart[1].toInt()][0] & B00000100);
        digitalWrite(strCommandPart[3].toInt(), arrEndValue[strCommandPart[1].toInt()][0] & B00000010);
        digitalWrite(strCommandPart[4].toInt(), arrEndValue[strCommandPart[1].toInt()][0] & B00000001);

        // Wait until it is time for the next step
        delay(strCommandPart[6].toInt());

        // See if there is a command waiting - if so, assume it is Break, so stop stepping
        if (Serial.available()) {
          break;
        }
      }

      // Only return OK when a Break didn't occur
      if (!Serial.available()) {
        Serial.println(F("OK"));
      }

      bytCommandResponse = enuCommandResponseNone;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // StepperMotorDisable
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("SMD") || strCommandPart[0] == F("STEPPERMOTORDISABLE")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);
      strCommandPart[3] = getWord(strCommand, 3);
      strCommandPart[4] = getWord(strCommand, 4);

      // Set the outputs low
      digitalWrite(strCommandPart[1].toInt(), LOW);
      digitalWrite(strCommandPart[2].toInt(), LOW);
      digitalWrite(strCommandPart[3].toInt(), LOW);
      digitalWrite(strCommandPart[4].toInt(), LOW);

      bytCommandResponse = enuCommandResponseOK;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // AnalogRead
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("AR") || strCommandPart[0] == F("ANALOGREAD") || strCommandPart[0] == F("ANALOGUEREAD")) {
      strCommandPart[1] = getWord(strCommand, 1);

      if (arrAnalogPinSetup[strCommandPart[1].toInt()] == enuPinSetupInputLatchingHigh || arrAnalogPinSetup[strCommandPart[1].toInt()] == enuPinSetupInputLatchingLow) {
        // Read the latch and return 1023 or 0 values
        if (arrAnalogPinSetup[strCommandPart[1].toInt()] == enuPinSetupInputLatchingHigh) {
          if (arrAnalogLatchTriggered[strCommandPart[1].toInt()]) {
            Serial.println(1023);
          } else {
            Serial.println(0);
          }
        }
        if (arrAnalogPinSetup[strCommandPart[1].toInt()] == enuPinSetupInputLatchingLow) {
          if (arrAnalogLatchTriggered[strCommandPart[1].toInt()]) {
            Serial.println(0);
          } else {
            Serial.println(1023);
          }
        }

        // Reset the latch
        arrAnalogLatchTriggered[strCommandPart[1].toInt()] = false;
      } else {
        // Read the pin and return the analogue value
        Serial.println(analogRead(strCommandPart[1].toInt()));
      }

      bytCommandResponse = enuCommandResponseNone;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // AnalogWrite
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("AW") || strCommandPart[0] == F("ANALOGUEWRITE") || strCommandPart[0] == F("ANALOGWRITE")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);
      strCommandPart[3] = getWord(strCommand, 3);

      // This pin is now a PWM output
      arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupOutputPWM;

      // Store the start value, end value, start time and duration
      arrStartValue[strCommandPart[1].toInt()][0] = arrCurrentValue[strCommandPart[1].toInt()][0];
      arrEndValue[strCommandPart[1].toInt()][0] = strCommandPart[2].toInt();
      arrStartTime[strCommandPart[1].toInt()][0] = millis();
      arrDuration[strCommandPart[1].toInt()][0] = strCommandPart[3].toInt();

      bytCommandResponse = enuCommandResponseOK;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // DigitalRead
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("DR") || strCommandPart[0] == F("DIGITALREAD")) {
      strCommandPart[1] = getWord(strCommand, 1);

      if (arrDigitalPinSetup[strCommandPart[1].toInt()] == enuPinSetupInputLatchingHigh || arrDigitalPinSetup[strCommandPart[1].toInt()] == enuPinSetupInputLatchingLow) {
        // Read the latch and return High or Low
        if (arrDigitalPinSetup[strCommandPart[1].toInt()] == enuPinSetupInputLatchingHigh) {
          if (arrDigitalLatchTriggered[strCommandPart[1].toInt()]) {
            Serial.println(F("High"));
          } else {
            Serial.println(F("Low"));
          }
        }
        if (arrDigitalPinSetup[strCommandPart[1].toInt()] == enuPinSetupInputLatchingLow) {
          if (arrDigitalLatchTriggered[strCommandPart[1].toInt()]) {
            Serial.println(F("Low"));
          } else {
            Serial.println(F("High"));
          }
        }

        // Reset the latch
        arrDigitalLatchTriggered[strCommandPart[1].toInt()] = false;
      } else {
        // Read the pin and return High or Low
        if (digitalRead(strCommandPart[1].toInt())) {
          Serial.println(F("High"));
        } else {
          Serial.println(F("Low"));
        }
      }

      bytCommandResponse = enuCommandResponseNone;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // DigitalWrite
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("DW") || strCommandPart[0] == F("DIGITALWRITE")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);

      // This pin is now an output
      arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupOutput;

      // Set the output to High or Low
      if (strCommandPart[2] == F("1") || strCommandPart[2] == F("HIGH")) {
        arrCurrentValue[strCommandPart[1].toInt()][0] = 255;
        digitalWrite(strCommandPart[1].toInt(), HIGH);
        bytCommandResponse = enuCommandResponseOK;
      }
      if (strCommandPart[2] == F("0") || strCommandPart[2] == F("LOW")) {
        arrCurrentValue[strCommandPart[1].toInt()][0] = 0;
        digitalWrite(strCommandPart[1].toInt(), LOW);
        bytCommandResponse = enuCommandResponseOK;
      }
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Acknowledge
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("A") || strCommandPart[0] == F("ACKNOWLEDGE")) {
      strCommandPart[1] = getWord(strCommand, 1);

      // Turn acknowledgements on or off
      if (strCommandPart[1] == F("1") || strCommandPart[1] == F("ON")) {
        booAcknowledge = true;
        bytCommandResponse = enuCommandResponseOK;
      }
      if (strCommandPart[1] == F("0") || strCommandPart[1] == F("OFF")) {
        booAcknowledge = false;
        bytCommandResponse = enuCommandResponseOK;
      }
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // AnalogPinMode
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("APM") || strCommandPart[0] == F("ANALOGUEPINMODE") || strCommandPart[0] == F("ANALOGPINMODE")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);

      if (strCommandPart[2] == F("I") || strCommandPart[2] == F("INPUT")) {
        // This pin is now an input
        arrAnalogPinSetup[strCommandPart[1].toInt()] = enuPinSetupInput;
        bytCommandResponse = enuCommandResponseOK;
      }

      if (strCommandPart[2] == F("LI") || strCommandPart[2] == F("LATCHINGINPUT")) {
        // This pin is now a latching input
        arrAnalogLatchTriggered[strCommandPart[1].toInt()] = false;
        bytCommandResponse = enuCommandResponseOK;

        if (analogRead(strCommandPart[1].toInt()) > 255) {
          arrAnalogPinSetup[strCommandPart[1].toInt()] = enuPinSetupInputLatchingLow;
        } else {
          arrAnalogPinSetup[strCommandPart[1].toInt()] = enuPinSetupInputLatchingHigh;
        }
      }
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // DigitalPinMode
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("DPM") || strCommandPart[0] == F("DIGITALPINMODE")) {
      strCommandPart[1] = getWord(strCommand, 1);
      strCommandPart[2] = getWord(strCommand, 2);

      arrCurrentValue[strCommandPart[1].toInt()][0] = 0;

      if (strCommandPart[2] == F("I") || strCommandPart[2] == F("INPUT")) {
        // This pin is now an input
        arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupInput;
        pinMode(strCommandPart[1].toInt(), INPUT);
        bytCommandResponse = enuCommandResponseOK;
      }

      if (strCommandPart[2] == F("IP") || strCommandPart[2] == F("INPUTPULLUP")) {
        // This pin is now a pullup input
        arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupInput;
        pinMode(strCommandPart[1].toInt(), INPUT_PULLUP);
        bytCommandResponse = enuCommandResponseOK;
      }

      if (strCommandPart[2] == F("LI") || strCommandPart[2] == F("LATCHINGINPUT")) {
        // This pin is now a latching input
        pinMode(strCommandPart[1].toInt(), INPUT);
        arrDigitalLatchTriggered[strCommandPart[1].toInt()] = false;
        bytCommandResponse = enuCommandResponseOK;

        if (digitalRead(strCommandPart[1].toInt())) {
          arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupInputLatchingLow;
        } else {
          arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupInputLatchingHigh;
        }
      }

      if (strCommandPart[2] == F("LIP") || strCommandPart[2] == F("LATCHINGINPUTPULLUP")) {
        // This pin is now a pullup latching input
        pinMode(strCommandPart[1].toInt(), INPUT_PULLUP);
        arrDigitalLatchTriggered[strCommandPart[1].toInt()] = false;
        bytCommandResponse = enuCommandResponseOK;

        if (digitalRead(strCommandPart[1].toInt())) {
          arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupInputLatchingLow;
        } else {
          arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupInputLatchingHigh;
        }
      }

      if (strCommandPart[2] == F("O") || strCommandPart[2] == F("OUTPUT")) {
        // This pin is now an output
        arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupOutput;
        pinMode(strCommandPart[1].toInt(), OUTPUT);
        bytCommandResponse = enuCommandResponseOK;
      }
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // MeccanoidInitialise
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("MNI") || strCommandPart[0] == F("MECCANOIDINITIALISE") || strCommandPart[0] == F("MECCANOIDINITIALIZE")) {
      strCommandPart[1] = getWord(strCommand, 1);

      meccanoidInitialise(strCommandPart[1].toInt());
      bytCommandResponse = enuCommandResponseNone;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Version
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("V") || strCommandPart[0] == F("VERSION")) {
      // Return model and version
      Serial.println(F(strModelAndVersion));
      bytCommandResponse = enuCommandResponseNone;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Break
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (strCommandPart[0] == F("B") || strCommandPart[0] == F("BREAK")) {
      // Break long-running commands (AnalogWrite, MeccanoidSetServoMotorPosition and StepperMotorStep)
      for (byte bytPin = bytLowestDigitalPin; bytPin <= bytHighestDigitalPin; bytPin++) {
        switch (arrDigitalPinSetup[bytPin]) {
          case enuPinSetupOutputPWM:
            {
              arrEndValue[bytPin][0] = arrCurrentValue[bytPin][0];
              break;
            }
          case enuPinSetupMeccanoid:
            {
              for (byte bytPosition = 0; bytPosition < 4; bytPosition++) {
                if (arrMeccanoidType[bytPin][bytPosition] == 1) {
                  // Get the servo position
                  arrCurrentValue[bytPin][bytPosition] = meccanoidGetServoMotorPosition(bytPin, bytPosition);
                  arrEndValue[bytPin][bytPosition] = arrCurrentValue[bytPin][bytPosition];
                }
              }
              break;
            }
          default:
            {
              // Do nothing
            }
        }
      }

      bytCommandResponse = enuCommandResponseOK;
    }

    // Acknowledge command?
    if (booAcknowledge && bytCommandResponse == enuCommandResponseOK) {
      // Return OK
      Serial.println(F("OK"));
    }
    if (bytCommandResponse == enuCommandResponseUnknown) {
      // Return Unknown
      Serial.println(F("Unknown command"));
    }
  }
}

//******************************************************************************************************************************************************************************************************
// HELPER FUNCTIONS
//******************************************************************************************************************************************************************************************************

String getWord(String strCommand, byte bytIndex)
{
  byte bytFound = 0;
  int arrIndex[] = {0, -1};
  byte bytMaxIndex = strCommand.length() - 1;

  for (byte bytPin = 0; bytPin <= bytMaxIndex && bytFound <= bytIndex; bytPin++) {
    if (strCommand.charAt(bytPin) == ' ' || bytPin == bytMaxIndex) {
      bytFound++;
      arrIndex[0] = arrIndex[1] + 1;
      arrIndex[1] = (bytPin == bytMaxIndex) ? bytPin + 1 : bytPin;
    }
  }

  return bytFound > bytIndex ? strCommand.substring(arrIndex[0], arrIndex[1]) : "";
}

//******************************************************************************************************************************************************************************************************
// MECCANOID FUNCTIONS
//******************************************************************************************************************************************************************************************************

void meccanoidInitialise(byte bytPin) {
  byte bytInput;
  byte bytPosition = 0;
  byte bytTimeout = 0;
  bool booAllPositionsDiscovered;

  // Is this pin already setup for Meccanoid use?
  if (arrDigitalPinSetup[bytPin] == enuPinSetupMeccanoid) {
    Serial.println(F("OK"));
    return;
  }

  // Send reset
  for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++) {
    arrMeccanoidData[bytPin][bytLoopPosition] = 0xFD;
    meccanoidCommunicate(bytPin, bytLoopPosition);
  }

  // Set positions and types as unassigned
  for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++) {
    arrMeccanoidData[bytPin][bytLoopPosition] = 0xFE;
    arrMeccanoidType[bytPin][bytLoopPosition] = 255;
  }

  do {
    bytInput = meccanoidCommunicate(bytPin, bytPosition);

    // If 0xFE is received, then the module exists so get its type
    if (bytInput == 0xFE) {
      arrMeccanoidData[bytPin][bytPosition] = 0xFC;
    }

    // If 0x01 is received, the module is a servo, so change its colour to black
    if (bytInput == 0x01 && arrMeccanoidType[bytPin][bytPosition] == 255) {
      arrMeccanoidData[bytPin][bytPosition] = 0xF0;
      arrMeccanoidType[bytPin][bytPosition] = 1;
    }

    // If 0x02 is received, the module is an LED
    if (bytInput == 0x02 && arrMeccanoidType[bytPin][bytPosition] == 255) {
      arrMeccanoidData[bytPin][bytPosition] = 0x00;
      arrMeccanoidType[bytPin][bytPosition] = 2;
    }

    // If 0x00 is received, there is no module at this or higher positions
    if (bytInput == 0x00 && bytPosition > 0) {
      if (arrMeccanoidType[bytPin][bytPosition - 1] != 255) {
        arrMeccanoidData[bytPin][bytPosition] = 0xFE;
        arrMeccanoidType[bytPin][bytPosition] = 0;
      }
    }

    // See if all positions have been discovered
    booAllPositionsDiscovered = true;
    for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++) {
      if (arrMeccanoidType[bytPin][bytLoopPosition] == 255) {
        booAllPositionsDiscovered = false;
      }
    }

    // Move to the next position
    bytPosition++;
    if (bytPosition == 4) {
      bytPosition = 0;
    }

    bytTimeout++;
  } while (!booAllPositionsDiscovered && bytTimeout != 0);

  if (bytTimeout == 0) {
    // This pin is now unused
    arrDigitalPinSetup[strCommandPart[1].toInt()] = enuPinSetupUnused;

    Serial.println(F("Timed out"));
  } else {
    for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++) {
      if (arrMeccanoidType[bytPin][bytLoopPosition] == 1) {
        // Set servo LED to black
        arrMeccanoidData[bytPin][bytLoopPosition] = 0xF0;

        // Get the servo position
        arrCurrentValue[bytPin][bytLoopPosition] = meccanoidGetServoMotorPosition(bytPin, bytLoopPosition);
        arrEndValue[bytPin][bytLoopPosition] = arrCurrentValue[bytPin][bytLoopPosition];
      }

      if (arrMeccanoidType[bytPin][bytLoopPosition] == 2) {
        // Set LED to black
        arrMeccanoidData[bytPin][bytLoopPosition] = 0x00;
        meccanoidCommunicate(bytPin, bytLoopPosition);
        arrMeccanoidData[bytPin][bytLoopPosition] = 0x40;
        meccanoidCommunicate(bytPin, bytLoopPosition);
      }
    }

    // This pin is now a Meccanoid pin
    arrDigitalPinSetup[bytPin] = enuPinSetupMeccanoid;

    Serial.println(F("OK"));
  }

  return;
}

byte meccanoidCommunicate(byte bytPin, byte bytPosition) {
  unsigned int intCheckSum;
  byte bytCheckSum;
  byte bytInput;

  // Send header
  meccanoidSendByte(bytPin, 0xFF);

  for (byte bytLoopPosition = 0; bytLoopPosition < 4; bytLoopPosition++) {
    // Send 4 data bytes
    meccanoidSendByte(bytPin, arrMeccanoidData[bytPin][bytLoopPosition]);
  }

  // Calculate checksum
  intCheckSum = arrMeccanoidData[bytPin][0] + arrMeccanoidData[bytPin][1] + arrMeccanoidData[bytPin][2] + arrMeccanoidData[bytPin][3];  // Ignore overflow
  intCheckSum = intCheckSum + (intCheckSum >> 8);                                                                                       // Right shift 8 places
  intCheckSum = intCheckSum + (intCheckSum << 4);                                                                                       // Left shift 4 places
  intCheckSum = intCheckSum & 0xF0;                                                                                                     // Mask off top nibble
  bytCheckSum = intCheckSum | bytPosition;

  // Send checksum
  meccanoidSendByte(bytPin, bytCheckSum);

  // Receive input
  bytInput = meccanoidReceiveByte(bytPin);

  delay(10);

  return bytInput;
}

void meccanoidSendByte(byte bytPin, byte bytData) {
  const unsigned int intMeccanoidBitDelay = 417;

  pinMode(bytPin, OUTPUT);
  digitalWrite(bytPin, LOW);
  delayMicroseconds(intMeccanoidBitDelay);                     // Start bit - 417us LOW

  for (byte bytMask = 00000001; bytMask > 0; bytMask <<= 1) {  // Iterate through bit mask
    if (bytData & bytMask) {                                   // If bitwise AND resolves to true

      digitalWrite(bytPin, HIGH);                              // Send 1

    } else {                                                   // If bitwise AND resolves to false

      digitalWrite(bytPin, LOW);                               // Send 0
    }
    delayMicroseconds(intMeccanoidBitDelay);                   // Delay
  }

  digitalWrite(bytPin, HIGH);
  delayMicroseconds(intMeccanoidBitDelay);                     // Stop bit - 417us HIGH

  digitalWrite(bytPin, HIGH);
  delayMicroseconds(intMeccanoidBitDelay);                     // Stop bit - 417us HIGH
}

byte meccanoidReceiveByte(byte bytPin) {
  byte bytTemp;
  bytTemp = 0;

  pinMode(bytPin, INPUT);

  delay(1.5);

  // Iterate through bit mask
  for (byte bytMask = 00000001; bytMask > 0; bytMask <<= 1) {
    if (pulseIn(bytPin, HIGH, 2500) > 400) {
      bytTemp = bytTemp | bytMask;
    }
  }

  return bytTemp;
}

byte meccanoidGetServoMotorPosition(byte bytPin, byte bytPosition) {
  // Get the servo position - keep getting returned position until two readings match and are non-zero
  byte bytReturnedValue = 0;
  byte bytLastReturnedValue = 0;
  
  do {
    bytLastReturnedValue = bytReturnedValue;
    bytReturnedValue = meccanoidCommunicate(bytPin, bytPosition);
  } while (bytReturnedValue == 0 || bytReturnedValue != bytLastReturnedValue);

  return bytReturnedValue;
}

