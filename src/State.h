/* State Definition */

#ifndef _State_H_
#define _State_H_

#include "NMEA2000.h"
#include "State.h"

enum PyPilotMode {
    compass,
    gps,
    wind,
    trueWind
} ;

enum RaymarineMode {
    Standby = 0,
    Compass = 0x40,
    Wind = 0x100,
    Track = 0x180,
    NoDrift = 0x181
};

class tState {
    protected:
        RaymarineMode pyPilot2RaymarineMode(PyPilotMode mode);
        PyPilotMode raymarine2PyPilotMode(RaymarineMode mode);

    public:

        bool engaged {false};
        PyPilotMode mode {PyPilotMode::compass};
        double headingCommandTrue {0.0};
        double headingCommandMagnetic {0.0};
        double variation;
        double heading {0.0};
        double rudderCommand {0.0};
        double rudderAngle {0.0};



        void setRaymarineMode(RaymarineMode mode);
        void setHeading(double heading);
        void setCommandHeadingMagnetic(double heading);
        void setCommandHeadingTrue(double heading);

        void printInfo();

        // Sending Information

        void sendPilotMode(tNMEA2000 *NMEA2000);
        void sendVesselHeading(tNMEA2000 *NMEA2000);
        void sendRudder(tNMEA2000 *NMEA2000);
        void sendLockedlHeading(tNMEA2000 *NMEA2000);

};


    



























#endif