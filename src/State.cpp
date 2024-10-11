#include <string.h>
#include "State.h"

#include <NMEA2000.h>



void tState::printInfo()
{
    Serial.println("--------------- State ------------------");
    Serial.print("Engaged: ");
    Serial.println(engaged.value);
    Serial.print("Mode: ");
    Serial.println(mode.value);
    Serial.print("Heading Command True: ");
    Serial.println(headingCommandTrue.value);
    Serial.print("Heading Command Magnetic: ");
    Serial.println(headingCommandMagnetic.value);
    Serial.print("Heading: ");
    Serial.println(heading.heading);
    Serial.print("Rudder Command: ");
    Serial.println(rudderCommand.command);
    Serial.print("Rudder Angle: ");
    Serial.println(rudderAngle.value);
}

