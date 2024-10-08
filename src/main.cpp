#include <Arduino.h>
#define ESP32_CAN_TX_PIN GPIO_NUM_16
#define ESP32_CAN_RX_PIN GPIO_NUM_17

#define V_PIN GPIO_NUM_36

#include <NMEA2000_esp32.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include "N2kDeviceList.h"
#include "GroupHandlers.h"
#include "State.h"

tN2kDeviceList *pN2kDeviceList;
#define START_DELAY_IN_S 8

// List here messages your device will transmit.
const char *onOffValues[] PROGMEM = {"Off", "On", "Error", "Unavailable"};
const char *steeringModeValues[] PROGMEM = {"Main Steering", "Non Follow Up Device", "Follow Up Device", "Standalone",
                                            "Heading Control", "Track Control", "", "Unavailable"};
const char *turnModeValues[] PROGMEM = {"Rudder limited", "Turn rate controller", "Radius controller", "", "", "", "", "Unavailable"};
const char *headingReferenceValues[] PROGMEM = {"true", "magnetic", "error", "unavailable"};
const char *rudderDirectionValues[] PROGMEM = {"No direction", "Starboard", "Port", "", "", "", "", "Unavailable"};
const char *distanceCalculationTypeValues[] PROGMEM = {"Great Circle", "Rhumb Line"};
const char *xTEModeValues[] PROGMEM = {"Autonomous", "Differential", "Estimated", "Simulator", "Manual"};
const unsigned long TransmitMessages[] PROGMEM = {126208L, // Request, command and "Reconocer?"
                                                  127245L,  // Rudder Angle every 40ms
                                                  127250L, // Rhumb 100 ms
                                                  65288L,   // Read Seatalk Alarm State
                                                  65379L,   // Read Pilot Mode
                                                  65360L,   // Read Pilot locked heading
                                                  // Perhaps 127251 rate of turn 100 ms and 127357 attitude at 100ms
                                                  0};
const unsigned long ReceiveMessages[] PROGMEM = { 126208L, // Request, Command and "Reconocer?"
                                                  127245L, // Rudder Angle
                                                  127250L, // * Rhumb - Vessel heading
                                                  127258L, // * Magnetic Variation
                                                  128259L, // Speed over water
                                                  129026L, // * Fast COG, SOG update
                                                  129029L, // * Position GNSS (Date, time, lat, lon)
                                                  129283L, // * XTE
                                                  129284L, // * Route information
                                                  129285L, // * Active Waypoint data
                                                  130306L, // * Wind data

                                                  // Other not in EVO-1 Document

  
                                                  127237L, // Heading Track Control
                                                  126720L, // Seatalk1 Pilot Mode
                                                  61184L, // Seatalk: Wireless Keypad  Control
                                                  65288L, // Seatalk Alarm
                                                  65379L, // Seatalk Pilot Mode
                                                  65360L, // Seatalk Pilot Locked Heading
                                                  0};

// ---  Example of using PROGMEM to hold Product ID.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const tNMEA2000::tProductInformation AutopilotProductInformation PROGMEM = {
    1300,                           // N2kVersion
    100,                            // Manufacturer's product code
    "87180-2-EN", // Manufacturer's Model ID
    "3.1.2",           // Manufacturer's Software version code
    "1.0.0",           // Manufacturer's Model version
    "00000001",                     // Manufacturer's Model serial code
    1,                              // CertificationLevel
    4                               // LoadEquivalency
};

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char AutopilotManufacturerInformation[] PROGMEM = "RAYMARINE, INC., fgorina@gmail.com";
const char AutopilotInstallationDescription1[] PROGMEM = "https://www.azimutmarine.es/docs/manuales/Raymarine/RAY_Evolution%20EV100,%20ACUx_ESP.pdf";
const char AutopilotInstallationDescription2[] PROGMEM = "Test";

const unsigned long AutopilotSerialNumber PROGMEM = 1;
const unsigned char AutopilotDeviceFunction = 150; // Autopilot
const unsigned char AutopilotDeviceClass = 40; // Steering and Control Surfaces
const uint16_t AutopilotManufacturerCode = 1851; // Raymarine
const unsigned char AutopilotIndustryGroup = 4; // Marine


bool verbose = false;
bool analyze = false;


tState &State=*(new tState());

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

// Functions for getting data

double rudderAngle(){
  return 1.0;
}

double heading(){
  return 3.141592;
}

//*****************************************************************************
void PrintUlongList(const char *prefix, const unsigned long *List)
{
  uint8_t i;
  if (List != 0)
  {
    Serial.print(prefix);
    for (i = 0; List[i] != 0; i++)
    {
      if (i > 0)
        Serial.print(", ");
      Serial.print(List[i]);
    }
    Serial.println();
  }
}

//*****************************************************************************
void PrintText(const char *Text, bool AddLineFeed = true)
{
  if (Text != 0)
    Serial.print(Text);
  if (AddLineFeed)
    Serial.println();
}

//*****************************************************************************
void PrintDevice(const tNMEA2000::tDevice *pDevice)
{
  if (pDevice == 0)
    return;

  Serial.println("----------------------------------------------------------------------");
  Serial.println(pDevice->GetModelID());
  Serial.print("  Source: ");
  Serial.println(pDevice->GetSource());
  Serial.print("  Manufacturer code:        ");
  Serial.println(pDevice->GetManufacturerCode());
  Serial.print("  Unique number:            ");
  Serial.println(pDevice->GetUniqueNumber());
  Serial.print("  Software version:         ");
  Serial.println(pDevice->GetSwCode());
  Serial.print("  Model version:            ");
  Serial.println(pDevice->GetModelVersion());
  Serial.print("  Manufacturer Information: ");
  PrintText(pDevice->GetManufacturerInformation());
  Serial.print("  Installation description1: ");
  PrintText(pDevice->GetInstallationDescription1());
  Serial.print("  Installation description2: ");
  PrintText(pDevice->GetInstallationDescription2());
  PrintUlongList("  Transmit PGNs :", pDevice->GetTransmitPGNs());
  PrintUlongList("  Receive PGNs  :", pDevice->GetReceivePGNs());
  Serial.println();
}

#define START_DELAY_IN_S 8
//*****************************************************************************
void ListDevices(bool force = false)
{
  static bool StartDelayDone = false;
  static int StartDelayCount = 0;
  static unsigned long NextStartDelay = 0;
  if (!StartDelayDone)
  { // We let system first collect data to avoid printing all changes
    if (millis() > NextStartDelay)
    {
      if (StartDelayCount == 0)
      {
        Serial.print("Reading device information from bus ");
        NextStartDelay = millis();
      }
      Serial.print(".");
      NextStartDelay += 1000;
      StartDelayCount++;
      if (StartDelayCount > START_DELAY_IN_S)
      {
        StartDelayDone = true;
        Serial.println();
      }
    }
    return;
  }
  if (!force && !pN2kDeviceList->ReadResetIsListUpdated())
    return;

  Serial.println();
  Serial.print("Max devices ");
  Serial.println(N2kMaxBusDevices);
  Serial.println("**********************************************************************");
  for (uint8_t i = 0; i < N2kMaxBusDevices; i++)
  {

    PrintDevice(pN2kDeviceList->FindDeviceBySource(i));
  }
}

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler NavigationDataScheduler(false, 100, 500);
tN2kSyncScheduler RudderAngleScheduler(false, 40, 510);
tN2kSyncScheduler BatConfScheduler(false, 5000, 520); // Non periodic

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen()
{
  return;
  // Start schedulers now.
  NavigationDataScheduler.UpdateNextTime();
  RudderAngleScheduler.UpdateNextTime();
 // BatConfScheduler.UpdateNextTime();
}

void setup_NMEA2000()
{
  NMEA2000.SetProductInformation(&AutopilotProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(AutopilotManufacturerInformation, AutopilotInstallationDescription1, AutopilotInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(AutopilotSerialNumber,   // Unique number. Use e.g. Serial number.
                                AutopilotDeviceFunction, // Device function=Autopìlot. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                AutopilotDeviceClass,  // Device class=Steering and Control Surfaces. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                AutopilotManufacturerCode, // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                AutopilotIndustryGroup // Industry Group
  );

  Serial.begin(115200);
  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on theTransmitTransmit  bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 25);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);  ttttrtt   // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(true); // Disable all msg forwarding to USB (=Serial)

  //  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple example, limit buffer size to 2, since we are only sending data
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  // Set Group Handlers




  NMEA2000.AddGroupFunctionHandler( new tN2kGroupFunctionHandlerForPGN65379(&NMEA2000, &State));
  NMEA2000.AddGroupFunctionHandler( new tN2kGroupFunctionHandlerForPGN127250(&NMEA2000, &State));
  NMEA2000.AddGroupFunctionHandler( new tN2kGroupFunctionHandlerForPGN127245(&NMEA2000, &State));
  NMEA2000.AddGroupFunctionHandler( new tN2kGroupFunctionHandlerForPGN65360(&NMEA2000, &State));

  // NMEA2000.SetOnOpen(OnN2kOpen);
  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);
  NMEA2000.Open();
}

void setup()
{

  // pinMode(V_PIN, INPUT_PULLUP);
  //  pinMode(ESP32_CAN_TX_PIN, OUTPUT);
  //  pinMode(ESP32_CAN_RX_PIN, INPUT_PULLUP);
  //   NMEA2000 Config

  setup_NMEA2000();
}

void SendN2kBattery()
{
  tN2kMsg N2kMsg;

  /* if (DCBatStatusScheduler.IsTime())
   {
       DCBatStatusScheduler.UpdateNextTime();

     SetN2kHeadingTrackControl(N2kMsg, NMEA2000.SetProductInformation(&AutopilotProductInformation);
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

typedef struct t_waypoint
{
  uint16_t id;
  char name[20];
  double latitude;
  double longitude;
} t_waypoint;

// Parse's que no hison a la llibreria

bool ParseN2kPGN129285(const tN2kMsg &N2kMsg, uint16_t &Start, uint16_t &nItems, uint16_t &Database, uint16_t &Route,
                       tN2kNavigationDirection &NavDirection, char *RouteName, size_t RouteNameBufSize, tN2kGenericStatusPair &SupplementaryData,
                       uint16_t wptArraySize, t_waypoint *waypoints)
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
  double lon;
  unsigned int nameSize = 20;

  for (int i = 0; i < min(nItems, wptArraySize); i++)
  {
    nameSize = 20;
    waypoints[i].id = N2kMsg.Get2ByteUInt(index);
    N2kMsg.GetVarStr(nameSize, waypoints[i].name, index);
    waypoints[i].latitude = N2kMsg.Get4ByteDouble(1.0e-7, index);
    waypoints[i].longitude = N2kMsg.Get4ByteDouble(1.0e-7, index);
  }

  return true;
}

bool parseN2kPGN126208(const tN2kMsg &N2kMsg, unsigned char &FunctionCode, uint32_t &PGN)
{
  int index = 0;

  if (N2kMsg.PGN != 126208)
  {
    return false;
  }
  FunctionCode = N2kMsg.GetByte(index);
  PGN = N2kMsg.Get3ByteUInt(index);
  return true;
}

// PGN 65379 in PGN 126208 command mode means SetMode

bool parseN2kPGN126208PGN65379(const tN2kMsg &N2kMsg, uint16_t &PilotMode)
{
  int index = 0;
  uint16_t ManufacturerCode;
  unsigned char IndustryCode;

  if (N2kMsg.PGN != 126208 && N2kMsg.GetByte(index) != 1 && N2kMsg.Get3ByteUInt(index) != 65379)
  {
    return false;
  }
  unsigned char priority = N2kMsg.GetByte(index);
  unsigned char nParams = N2kMsg.GetByte(index);

  // Here begin the actual parameters of 65379
  unsigned ip = N2kMsg.GetByte(index);
  if (ip != 1)
  {
    return false;
  }
  ManufacturerCode = N2kMsg.Get2ByteUInt(index);
  if (ManufacturerCode != 1851)
  { // Raymarine
    return false;
  }
  ip = N2kMsg.GetByte(index);
  if (ip != 3)
  {
    return false;
  }
  IndustryCode = N2kMsg.GetByte(index); // Marine
  if (IndustryCode != 4)
  {
    return false;
  }
  ip = N2kMsg.GetByte(index);
  if (ip != 4)
  {
    return false;
  }

  PilotMode = N2kMsg.Get2ByteInt(index);

  // 0 -> STANDBY, 64 -> Auto compass, 256 -> Vane, Wind  384 -> Track, 385 -> No Drift, COG Referenced

  return 1;
}

// PGN 65360 in PGN 126208 command mode means Set Locked heading

bool parseN2kPGN126208PGN65360(const tN2kMsg &N2kMsg, double &PilotCourse)
{
  int index = 0;
  uint16_t ManufacturerCode;
  unsigned char IndustryCode;

  if (N2kMsg.PGN != 126208 && N2kMsg.GetByte(index) != 1 && N2kMsg.Get3ByteUInt(index) != 65360)
  {
    return false;
  }
  unsigned char priority = N2kMsg.GetByte(index);
  unsigned char nParams = N2kMsg.GetByte(index);

  // Here begin the actual parameters of 65379
  unsigned char ip;
  ip = N2kMsg.GetByte(index);
  if (ip != 1)
  {
    return false;
  }
  ManufacturerCode = N2kMsg.Get2ByteUInt(index);
  if (ManufacturerCode != 1851)
  { // Raymarine
    return false;
  }
  ip = N2kMsg.GetByte(index);
  if (ip != 3)
  {
    return false;
  }
  IndustryCode = N2kMsg.GetByte(index); // Marine
  if (IndustryCode != 4)
  {
    return false;
  }
  ip = N2kMsg.GetByte(index);
  if (ip != 4)
  {
    return false;
  }
  PilotCourse = N2kMsg.Get2ByteDouble(0.0001, index);
  return true;
}

// Here is SetWindCourse

bool parseN2kPGN126208PGN65345(const tN2kMsg &N2kMsg, double &WindDatum)
{
  int index = 0;
  uint16_t ManufacturerCode;
  unsigned char IndustryCode;

  if (N2kMsg.PGN != 126208 && N2kMsg.GetByte(index) != 1 && N2kMsg.Get3ByteUInt(index) != 65360)
  {
    return false;
  }
  unsigned char priority = N2kMsg.GetByte(index);
  unsigned char nParams = N2kMsg.GetByte(index);

  // Here begin the actual parameters of 65379
  unsigned char ip;
  ip = N2kMsg.GetByte(index);
  if (ip != 1)
  {
    return false;
  }
  ManufacturerCode = N2kMsg.Get2ByteUInt(index);
  if (ManufacturerCode != 1851)
  { // Raymarine
    return false;
  }
  ip = N2kMsg.GetByte(index);
  if (ip != 3)
  {
    return false;
  }
  IndustryCode = N2kMsg.GetByte(index); // Marine
  if (IndustryCode != 4)
  {
    return false;
  }
  ip = N2kMsg.GetByte(index);

  WindDatum = N2kMsg.Get2ByteDouble(0.0001, index);
  return true;
}

/*
void handle126208Command(const tN2kMsg &N2kMsg)
{
  unsigned char FunctionCode;
  uint32_t PGN;
  uint16_t PilotMode;
  double PilotHeading;
  double WindDatum;

  if (!parseN2kPGN126208(N2kMsg, FunctionCode, PGN))
  {
    return;
  }
  if (FunctionCode == 1)
  { // Command. Will add Request later
    switch (PGN)
    {
    case 65379: // SetMode
      if (parseN2kPGN126208PGN65379(N2kMsg, PilotMode))
      {
        Serial.print("Setting Autopilot Mode to: ");
        Serial.println(PilotMode);
      }
      break;

    case 65360: // Set locked heading
      if (parseN2kPGN126208PGN65360(N2kMsg, PilotHeading))
      {
        Serial.print("Setting Autopilot Heading to: ");
        Serial.println(PilotHeading);
      }
      break;

    case 65345: // Set WindDatum
      if (parseN2kPGN126208PGN65345(N2kMsg, WindDatum))
      {
        Serial.print("Setting Wind Datum to: ");
        Serial.println(WindDatum);
      }
      break;
    }
  }
}
*/
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
    Serial.print("Received Heading Track control Packet (127237)");
    Serial.print(" from: ");
    Serial.println(N2kMsg.Source);
    if (!verbose)
    {
      return;
    }
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
    Serial.print("Received Navigation Data Packet (129284)");
    Serial.print(" from: ");
    Serial.println(N2kMsg.Source);
    if (!verbose)
    {
      return;
    }
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
    Serial.print("Received XTE  Packet (129283)");
    Serial.print(" from: ");
    Serial.println(N2kMsg.Source);
    if (!verbose)
    {
      return;
    }
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
    Serial.print("Received Route Packet (129285)");
    Serial.print(" from: ");
    Serial.println(N2kMsg.Source);
    if (!verbose)
    {
      return;
    }
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

    for (int i = 0; i < nItems; i++)
    {
      Serial.print("wp ");
      Serial.print(waypoints[i].id);
      Serial.print(" ");
      Serial.print(waypoints[i].name);
      Serial.print(" Lat ");
      Serial.print(waypoints[i].latitude);
      Serial.print(" Lon ");
      Serial.println(waypoints[i].longitude);
    }
  }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
  if (!analyze)
  {
    return;
  }

  switch (N2kMsg.PGN)
  {
  
   case 127245:
    Serial.println("Received rudder angle info (127245  )");
    break;

  case 127250:
    Serial.println("Received heading (127250)");
    break;

  case 127258:
    Serial.println("Received Magnetic Variation (127258)");
    break;

  case 127259:
    Serial.println("Received water speed (127259)");
    break;

  case 129026:
    Serial.println("Received COG/SOG (129026)");
    break;

  case 129029:
    Serial.println("Received GNSS/position (129029)");
    break;

  case 129283:
    handleXTE(N2kMsg);
    break;

case 129284:
    handleNavigationInfo(N2kMsg);
    break;
case 129285:
    handleRouteInfo(N2kMsg);
    break;
case 130306:
    Serial.println("Received wind data (130306)");
    break;

  case 127237:
    handleHeadingTrackControl(N2kMsg);
    break;

  case 126720:
    Serial.println("Received key command");
    break;
 

  case 61184:
    Serial.println("Received remote command");
    break;


  default:
    Serial.print("Received unknown message ");
    Serial.println(N2kMsg.PGN);
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

void loop()
{

  // SendN2kBattery();
  NMEA2000.ParseMessages();
  ListDevices();

  if (Serial.available() > 0)
  {
    int command = Serial.read();
    Serial.print("Received key : ");
    Serial.println(command);

    switch (command)
    {

    case 'i':
    case 'I':

      State.printInfo();
      break;

    case 108:
    case 76:

      ListDevices(true);
      break;

    case 84:
    case 116:

      NMEA2000.EnableForward(true);
      Serial.println("Tracking");
      break;

      /* case 83:
       case 115:
         sendAutopilotMessage();
         StickCP2.Display.clear();
         StickCP2.Display.setCursor(10, 30);
         StickCP2.Display.printf("Sent autopilot");
         Serial.println("Sent autopilot message");
         break;
   */
    case 86:
    case 118: // v and V
      verbose = !verbose;
      break;

    case 65:
    case 97:
      analyze = !analyze;
      if(analyze){
        Serial.println("Analyzing");

      }else{
        Serial.println("Not Analyzing");
      }
      break;

    case 27:

      NMEA2000.EnableForward(false);
      Serial.println("NOT Tracking");
      break;
    }
  }
}