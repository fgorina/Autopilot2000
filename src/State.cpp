#include <string.h>
#include "State.h"
#include "NMEA2000.h"

RaymarineMode tState::pyPilot2RaymarineMode(PyPilotMode mode)
{
    return RaymarineMode::Compass;
}

PyPilotMode tState::raymarine2PyPilotMode(RaymarineMode mode)
{
    switch (mode)
    {
    case RaymarineMode::Standby:
        return this->mode;

    case RaymarineMode::Compass:
        return PyPilotMode::compass;

    case RaymarineMode::Wind:
        return PyPilotMode::wind;

    case RaymarineMode::Track:
        return PyPilotMode::compass;

    case RaymarineMode::NoDrift:
        return PyPilotMode::gps;

    default:
        Serial.print("Error en Raymarine Pilot Mode : ");
        Serial.println(mode);
        return PyPilotMode::compass;
    }
}

void tState::setRaymarineMode(RaymarineMode mode)
{

    if (mode == RaymarineMode::Standby)
    {
        this->engaged = false;
        Serial.println("Received Standby. Setting Engaged = false");
        // TODO: Send data to Pypilot
    }
    else
    {
        this->mode = raymarine2PyPilotMode(mode);
        this->engaged = true;

        Serial.print("Received ");
        Serial.print(mode);
        Serial.print(" Seting mode to "); Serial.print(this->mode); Serial.println(" and engaged = true");
        // TODO: Send data to PyPilot
    }
}

void tState::setHeading(double heading)
{
    this->heading = heading;

    // TODO: Must be updated from Autopilot
}

void tState::setCommandHeadingTrue(double heading)
{
    this->headingCommandTrue = heading;

    // TODO: Must besent to Autopilot
}

void tState::setCommandHeadingMagnetic(double heading)
{
    this->headingCommandMagnetic = heading;

    // TODO: Must besent to Autopilot
}

void tState::printInfo()
{
    Serial.println("--------------- State ------------------");
    Serial.print("Engaged: ");
    Serial.println(engaged);
    Serial.print("Mode: ");
    Serial.println(mode);
    Serial.print("Heading Command True: ");
    Serial.println(headingCommandTrue);
    Serial.print("Heading Command Magnetic: ");
    Serial.println(headingCommandMagnetic);
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.print("Rudder Command: ");
    Serial.println(rudderCommand);
    Serial.print("Rudder Angle: ");
    Serial.println(rudderAngle);
}


void tState::sendPilotMode(tNMEA2000 *NMEA2000){

    tN2kMsg N2kMsg ;
    N2kMsg.SetPGN(65379L);
    N2kMsg.AddByte(0x3B);   
    N2kMsg.AddByte(0x47);

    uint16_t rmode = pyPilot2RaymarineMode(mode);
    N2kMsg.Add2ByteUInt(rmode); // mode
    N2kMsg.Add2ByteUInt(0L);    // submode
    N2kMsg.AddByte(0); // Pilot Mode Data
    N2kMsg.AddByte(0); // Reserved

    NMEA2000->SendMsg(N2kMsg);
}

void tState::sendVesselHeading(tNMEA2000 *NMEA2000){
    Serial.print("Sending vessel heading of "); Serial.println(heading);
    tN2kMsg N2kMsg ;
    N2kMsg.SetPGN(127250);
    double radVesselHeading = heading / 180.0 * 3.141592;

    N2kMsg.Add2ByteUDouble(radVesselHeading, 0.0001);
    N2kMsg.Add2ByteUDouble(-1E9, 0.0001);
    N2kMsg.Add2ByteUDouble(-1E9, 0.0001);
    N2kMsg.AddByte(1);  // Magnetioc (true is 0)

    NMEA2000->SendMsg(N2kMsg);
}

void tState::sendRudder(tNMEA2000 *NMEA2000){
    Serial.print("Sending rudder angle of "); Serial.println(rudderAngle);
    tN2kMsg N2kMsg ;
    N2kMsg.SetPGN(127245);
    double radRudderAngle = rudderAngle / 180.0 * 3.141592;

    N2kMsg.AddByte(0);  // No rudder order        
    N2kMsg.Add2ByteUDouble(-1E9, 0.0001); // Rudder Order
    N2kMsg.Add2ByteUDouble(radRudderAngle, 0.0001);
    N2kMsg.Add2ByteUInt(0); // Reserved
    
    NMEA2000->SendMsg(N2kMsg);
}

void tState::sendLockedlHeading(tNMEA2000 *NMEA2000){
    Serial.print("Sending locked heading of "); Serial.println(heading);
    tN2kMsg N2kMsg ;
    N2kMsg.SetPGN(65360);
    double radLockedHeadingTrue = headingCommandTrue / 180.0 * 3.141592;
    double radLockedHeadingMagnetic = headingCommandMagnetic / 180.0 * 3.141592;
  
    N2kMsg.AddByte(0x3B);       // Raymarine, Marine
    N2kMsg.AddByte(0x47);

    N2kMsg.AddByte(1); // SID
    N2kMsg.Add2ByteDouble(radLockedHeadingTrue, 0.0001);
    N2kMsg.Add2ByteDouble(radLockedHeadingMagnetic, 0.0001);
  
    N2kMsg.AddByte(0);  // Reserved

    NMEA2000->SendMsg(N2kMsg);
}