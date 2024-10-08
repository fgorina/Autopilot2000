/* State Definition */

#ifndef _State_H_
#define _State_H_

enum PyPilotMode {
    Compass,
    GPS,
    Wind,
    TrueWind
} ;

enum RaymarineMode {
    Standby = 0,
    Compass = 0x40,
    Wind = 0x100,
    Track = 0x180,
    NoDrift = 0x181

};

typedef struct tState {

    bool engaged;
    PyPilotMode mode;
    double headingComand;
    double heading;
    double rudderCommand;
    double rudderAngle;
} tState;

// TODO: Put conversion table between nodes

RaymarineMode pyPilot2RaymarineMode(PyPilotMode mode){
    return RaymarineMode::Compass;    
}

PyPilotMode raymarine2PyPilotMode(RaymarineMode mode){
    return PyPilotMode::Compass;
}




























#endif