/* State Definition */

#ifndef _State_H_
#define _State_H_

#include "NMEA2000.h"
#include "N2kTypes.h"
#include "PyTypes.h"
#include "State.h"



class tState {
    protected:
    
    public:

        // Autopilot Data
        
        tHeadingData heading {when: 0, origin: tDataOrigin::NO_DATA, reference: tN2kHeadingReference::N2khr_Unavailable, heading: 0.0};   // In Radians
        tDoubleData deviation {when: 0, origin: tDataOrigin::NO_DATA, value:0.0}; // In degrees
        tDoubleData variation {when: 0, origin: tDataOrigin::NO_DATA, value:0.0};   // In degrees
        tDoubleData rudderAngle {when: 0, origin: tDataOrigin::NO_DATA, value: 0.0}; // rudder.angle

        // RW , Commands and data
        tModeData mode {when: 0, origin: tDataOrigin::NO_DATA, value:tPyPilotMode::compass}; // ap.mode
        tBoolData engaged {when: 0, origin: tDataOrigin::NO_DATA, value: false};   // ap.enabled
        tDoubleData headingCommandTrue {when: 0, origin: tDataOrigin::NO_DATA, value:0.0}; //In degrees
        tDoubleData headingCommandMagnetic {when: 0, origin: tDataOrigin::NO_DATA, value:0.0};    // ap.heading_command In degrees
        tRudderCommandData rudderCommand {when: 0, origin: tDataOrigin::NO_DATA,  direction: tN2kRudderDirectionOrder::N2kRDO_NoDirectionOrder , command: 0.0};

        tTackStateData tackState {when: 0, origin: tDataOrigin::NO_DATA, value: tTackState::TACK_NONE}; //ap.tack.state
        tTackDirectionData tackDirection {when: 0, origin: tDataOrigin::NO_DATA, value:tTackDirection::TACKING_NONE}; // ap.tack.direction

        // Servo Data

        tDoubleData servoVoltage {when: 0, origin: tDataOrigin::NO_DATA, value:0.0}; // servo.voltage
        tDoubleData servoAmpHr {when: 0, origin: tDataOrigin::NO_DATA, value:0.0}; //servo.amp_hours
        tDoubleData servoControllerTemp {when: 0, origin: tDataOrigin::NO_DATA, value:0.0}; // servo.controller_temp      
        tDoubleData servoPosition {when: 0, origin: tDataOrigin::NO_DATA, value:0.0}; // servo.position


        

        void printInfo();

      

};

#endif