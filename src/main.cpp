#include <Arduino.h>

#define ESP32_CAN_RX_PIN GPIO_NUM_17

#define V_PIN GPIO_NUM_36

#include <NMEA2000_esp32.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
// List here messages your device will transmit.
const char *onOffValues[] PROGMEM = {"Off", "On", "Error", "Unavailable"};
const char *steeringModeValues[] PROGMEM = {"Main Steering", "Non Follow Up Device", "Follow Up Device", "Standalone",
                                            "Heading Control", "Track Control", "", "Unavailable"};
const char *turnModeValues[] PROGMEM = {"Rudder limited", "Turn rate controller", "Radius controller", "", "", "", "", "Unavailable"};
const char *headingReferenceValues[] PROGMEM = {"true", "magnetic", "error", "unavailable"};
const char *rudderDirectionValues[] PROGMEM = {"No direction", "Starboard", "Port", "", "", "", "", "Unavailable"};
const char *distanceCalculationTypeValues[] PROGMEM = {"Great Circle", "Rhumb Line"};
const char *xTEModeValues[] PROGMEM = {"Autonomous", "Differential", "Estimated", "Simulator", "Manual"};
const unsigned long TransmitMessages[] PROGMEM = {127506L, 127508L, 127513L, 0};
const unsigned long ReceiveMessages[] PROGMEM = {127237L, 129284L, 129283L, 129285L, 0};

// ---  Example of using PROGMEM to hold Product ID.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const tNMEA2000::tProductInformation AutopilotProductInformation PROGMEM = {
    2100,                           // N2kVersion
    100,                            // Manufacturer's product code
    "Autopilot bridge for PyPilot", // Manufacturer's Model ID
    "0.0.1 (2024-09-29)",           // Manufacturer's Software version code
    "0.0.1 (2024-09-29)",           // Manufacturer's Model version
    "00000001",                     // Manufacturer's Model serial code
    0,                              // SertificationLevel
    1                               // LoadEquivalency
};

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char AutopilotManufacturerInformation[] PROGMEM = "Francisco Gorina, fgorina@gmail.com";
const char AutopilotInstallationDescription1[] PROGMEM = "Just install it";
const char AutopilotInstallationDescription2[] PROGMEM = "Test";

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler DCBatStatusScheduler(false, 1500, 500);
tN2kSyncScheduler DCStatusScheduler(false, 1500, 510);
tN2kSyncScheduler BatConfScheduler(false, 5000, 520); // Non periodic

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen()
{
  // Start schedulers now.
  DCBatStatusScheduler.UpdateNextTime();
  DCStatusScheduler.UpdateNextTime();
  BatConfScheduler.UpdateNextTime();
}

void setup()
{

  pinMode(V_PIN, INPUT_PULLUP);
  // pinMode(ESP32_CAN_TX_PIN, OUTPUT);
  // pinMode(ESP32_CAN_RX_PIN, INPUT_PULLUP);
  //  NMEA2000 Config

  NMEA2000.SetProductInformation(&AutopilotProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(AutopilotManufacturerInformation, AutopilotInstallationDescription1, AutopilotInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(1,   // Unique number. Use e.g. Serial number.
                                150, // Device function=Autopìlot. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                40,  // Device class=Steering and Control Surfaces. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  Serial.begin(115200);
  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on theTransmitTransmit  bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 25);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);  ttttrtt   // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  // NMEA2000.EnableForward(false);                      // Disable all msg forwarding to USB (=Serial)

  //  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple example, limit buffer size to 2, since we are only sending data
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();
}

void SendN2kBattery()
{
  tN2kMsg N2kMsg;

  /* if (DCBatStatusScheduler.IsTime())
   {
       DCBatStatusScheduler.UpdateNextTime();

     SetN2kHeadingTrackControl(N2kMsg,
     N2kOnOff_Unavailable,
     N2kOnOff_Unavailable,
     N2kOnOff_Unavailable,
     N2kOnOff_Unavailable,
     N2kSM_HeadingControl,
     N2kTM_Unavailable,
     N2khr_magnetic,
     N2kRDO_Unavailable, 0.0, 3.151592, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

     NMEA2000.SendMsg(N2kMsg);
   }
   */

  /*if ( DCStatusScheduler.IsTime() ) {
    DCStatusScheduler.UpdateNextTime();
    SetN2kDCStatus(N2kMsg,1,1,N2kDCt_Battery,56,92,38500,0.012);
    NMEA2000.SendMsg(N2kMsg);
  }

  if ( BatConfScheduler.IsTime() ) {
    BatConfScheduler.UpdateNextTime();
    SetN2kBatConf(N2kMsg,1,N2kDCbt_Gel,N2kDCES_Yes,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(420),53,1.251,75);
    NMEA2000.SendMsg(N2kMsg);
  }
  */
}
void loop()
{

  SendN2kBattery();
  NMEA2000.ParseMessages();
}

typedef struct t_waypoint {
  uint16_t id;
  char name[20];
  double latitude;
  double longitude;
} t_waypoint;

// No hi es a la llibreria

bool ParseN2kPGN129285(const tN2kMsg &N2kMsg, uint16_t &Start, uint16_t &nItems, uint16_t &Database, uint16_t &Route,
                       tN2kNavigationDirection &NavDirection, char *RouteName, size_t RouteNameBufSize, tN2kGenericStatusPair &SupplementaryData,
                       uint16_t wptArraySize, t_waypoint* waypoints )
{

  if (N2kMsg.PGN != 129285L)
    return false;

  int index = 0;
  unsigned char c;
  Start = N2kMsg.Get2ByteUInt(index);
  nItems = N2kMsg.Get2ByteUInt(index);
  Database = N2kMsg.Get2ByteUInt(index);
  Route = N2kMsg.Get2ByteUInt(index);

  c = N2kMsg.GetByte(index);
  // Use flags to set values
  SupplementaryData = tN2kGenericStatusPair((c & 0x18) >> 3);
  NavDirection = tN2kNavigationDirection(c && 0x07);
  N2kMsg.GetVarStr(RouteNameBufSize, RouteName, index);
  c = N2kMsg.GetByte(index); // Reserved
 
  uint16_t wpid;
  char wpname[20] = "";
  double lat;
  double lon;Waypoint

  for (int i = 0; i < min(nItems, wptArraySize); i++)
  {
    nameSize = 20;
    waypoints[i].id = N2kMsg.Get2ByteUInt(index);
    N2kMsg.GetVarStr(nameSize, waypoints[i].name,  index);
    waypoints[i].latitude = N2kMsg.Get4ByteDouble(1.0e-7, index);
    waypoints[i].longitude = N2kMsg.Get4ByteDouble(1.0e-7, index);

  }

  return true;
}

void handleHeadingTrackControl(const tN2kMsg &N2kMsg)
{
  tN2kOnOff RudderLimitExceeded;
  tN2kOnOff OffHeadingLimitExceeded;
  tN2kOnOff OffTrackLimitExceeded;
  tN2kOnOff Override;
  tN2kSteeringMode SteeringMode;
  tN2kTurnMode TurnMode;
  tN2kHeadingReference HeadingReference;
  tN2kRudderDirectionOrder CommandedRudderDirection;
  double CommandedRudderAngle;
  double HeadingToSteerCourse;
  double Track;
  double RudderLimit;
  double OffHeadingLimit;
  double RadiusOfTurnOrder;
  double RateOfTurnOrder;
  double OffTrackLimit;
  double VesselHeading;

  if (ParseN2kHeadingTrackControl(N2kMsg, RudderLimitExceeded, OffHeadingLimitExceeded, OffTrackLimitExceeded, Override, SteeringMode,
                                  TurnMode, HeadingReference, CommandedRudderDirection, CommandedRudderAngle, HeadingToSteerCourse, Track, RudderLimit,
                                  OffHeadingLimit, RadiusOfTurnOrder, RateOfTurnOrder, OffTrackLimit, VesselHeading))

  {
    Serial.println("Received Heading Track control Packet (127237)");
    Serial.print("Rudder limit exceeded: ");
    Serial.println(onOffValues[RudderLimitExceeded]);
    Serial.print("Off heading limit exceeded: ");
    Serial.println(onOffValues[OffHeadingLimitExceeded]);
    Serial.print("Off Track limit exceeded: ");
    Serial.println(onOffValues[OffTrackLimitExceeded]);
    Serial.print("Override: ");
    Serial.println(onOffValues[Override]);

    Serial.print("Steering Mode: ");
    Serial.println(steeringModeValues[SteeringMode]);
    Serial.print("Turn Mode: ");
    Serial.println(turnModeValues[TurnMode]);
    Serial.print("Heading Reference: ");
    Serial.println(headingReferenceValues[HeadingReference]);
    Serial.print("Rudder Direction: ");
    Serial.println(rudderDirectionValues[CommandedRudderDirection]);

    Serial.print("Command Rudder Angle: ");
    Serial.println(CommandedRudderAngle);
    Serial.print("Heading to steer course: (");
    Serial.print(HeadingToSteerCourse / 3.141592 * 180);
    Serial.print(") ");
    Serial.println(HeadingToSteerCourse);
    Serial.print("Track: ");
    Serial.println(Track);

    Serial.print("Rudder limit: ");
    Serial.println(RudderLimit);

    Serial.print("Off Heading limit: ");
    Serial.println(OffHeadingLimit);

    Serial.print("Radius of Turn Order: ");
    Serial.println(RadiusOfTurnOrder);

    Serial.print("Rate of Turn Order: ");
    Serial.println(RateOfTurnOrder);

    Serial.print("Off Track limit: ");
    Serial.println(OffTrackLimit);
    Serial.print("Vessel heading: ");
    Serial.println(VesselHeading);
    Serial.println("------------------------------------------------------------------------------");
  }
}

void handleNavigationInfo(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double DistanceToWaypoint;
  tN2kHeadingReference BearingReference;
  bool PerpendicularCrossed;
  bool ArrivalCircleEntered;
  tN2kDistanceCalculationType CalculationType;
  double ETATime;
  int16_t ETADate;
  double BearingOriginToDestinationWaypoint;
  double BearingPositionToDestinationWaypoint;
  uint32_t OriginWaypointNumber;
  uint32_t DestinationWaypointNumber;
  double DestinationLatitude;
  double DestinationLongitude;
  double WaypointClosingVelocity;

  if (ParseN2kNavigationInfo(N2kMsg, SID, DistanceToWaypoint, BearingReference, PerpendicularCrossed, ArrivalCircleEntered, CalculationType,
                             ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint,
                             OriginWaypointNumber, DestinationWaypointNumber, DestinationLatitude, DestinationLongitude, WaypointClosingVelocity))
  {
    Serial.println("Received Navigation Data Packet (129284)");
    Serial.print("SID: ");
    Serial.println(SID);
    Serial.print("DistanceToWaypoint: ");
    Serial.println(DistanceToWaypoint);
    Serial.print("BearingReference: ");
    Serial.println(headingReferenceValues[BearingReference]);
    Serial.print("PerpendicularCrossed: ");
    Serial.println(PerpendicularCrossed);
    Serial.print("ArrivalCircleEntered: ");
    Serial.println(ArrivalCircleEntered);
    Serial.print("CalculationType: ");
    Serial.println(CalculationType);
    Serial.print("ETATime: ");
    Serial.println(ETATime);
    Serial.print("ETADate: ");
    Serial.println(ETADate);
    Serial.print("BearingOriginToDestinationWaypoint: ");
    Serial.println(BearingOriginToDestinationWaypoint);
    Serial.print("BearingPositionToDestinationWaypoint: ");
    Serial.println(BearingPositionToDestinationWaypoint);
    Serial.print("OriginWaypointNumber: ");
    Serial.println(OriginWaypointNumber);
    Serial.print("DestinationWaypointNumber: ");
    Serial.println(DestinationWaypointNumber);
    Serial.print("DestinationLatitude: ");
    Serial.println(DestinationLatitude);
    Serial.print("DestinationLongitude: ");
    Serial.println(DestinationLongitude);
    Serial.print("WaypointClosingVelocity: ");
    Serial.println(WaypointClosingVelocity);
    Serial.println("------------------------------------------------------------------------------");
  }
}
void handleXTE(const tN2kMsg &N2kMsg)
{

  unsigned char SID;
  tN2kXTEMode XTEMode;
  bool NavigationTerminated;
  double XTE;

  if (ParseN2kXTE(N2kMsg, SID, XTEMode, NavigationTerminated, XTE))
  {
    Serial.println("Received XTE  Packet (129283)");
    Serial.print("    SID: ");
    Serial.println(SID);
    Serial.print("    XTEMode: ");
    Serial.println(xTEModeValues[XTEMode]);
    Serial.print("    NavigationTerminated: ");
    Serial.println(NavigationTerminated);
    Serial.print("    XTE: ");
    Serial.println(XTE);
    Serial.println("------------------------------------------------------------------------------");
  }
}
void handleRouteInfo(const tN2kMsg &N2kMsg)
{

  uint16_t Start;
  uint16_t nItems;
  uint16_t Database;
  uint16_t Route;
  t_waypoint waypoints[10];

  tN2kNavigationDirection NavDirection;
  char RouteName[21] = "";
  tN2kGenericStatusPair SupplementaryData;

  if (ParseN2kPGN129285(N2kMsg, Start, nItems, Database, Route, NavDirection, RouteName, 20, SupplementaryData, 10, waypoints))
  {
    Serial.println("Received Route Packet (129285)");
    Serial.print("Start: ");
    Serial.println(Start);
    Serial.print("nItems: ");
    Serial.println(nItems);
    Serial.print("Database: ");
    Serial.println(Database);
    Serial.print("Route: ");
    Serial.println(Route);
    Serial.print("NavDirection: ");
    Serial.println(NavDirection);
    Serial.print("RouteName: ");
    Serial.println(RouteName);
    Serial.print("SupplementaryData: ");
    Serial.println(SupplementaryData);

    for(int i = 0; i < nItems; i++){
      Serial.print("wp "); Serial.print(waypoints[i].id); Serial.print(" "); Serial.print(waypoints[i].name);
      Serial.print(" Lat "); Serial.print(waypoints[i].latitude); Serial.print(" Lon "); Serial.println(waypoints[i].longitude);
    }
  }
}
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{

  switch (N2kMsg.PGN)
  {
  case 127237:
    handleHeadingTrackControl(N2kMsg);
    break;

  case 129284:
    handleNavigationInfo(N2kMsg);
    break;

  case 129285:
    handleRouteInfo(N2kMsg);
    break;

  case 129283:
    handleXTE(N2kMsg);
    break;
  }
}

/* Here we have an example. Don't know how to send the nulls

PGN: 127237, Source: 1, Destination: 255
Rudder Limit: 3 - Unavailable
Heading Limit: 3 - Unavailable
Track Limit: 3 - Unavailable
Overide: 3 - Unavailable
Steering Mode: 1 - NonFollowUpDevice
Turn Mode: 7 - Unavailable
Heading Mode: 3 - Unavailable
Rudder Direction 7 -> Unavailable
Rudder Angle: null ???
Heading To Steer: 218.233911 ??? Is in degrees!!!
Track: null ???
Rudder Limit: null >???
Heading Limit: null ???
Turn Radius: null ???
Turn Rate: null ???
Off Track Limit: null ???
Vessel Heading: n

*/