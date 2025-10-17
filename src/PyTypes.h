#ifndef _PyTypes_H_
#define _PyTypes_H_


typedef enum  {
    compass = 0,
    gps = 1,
    wind = 2,
    trueWind = 3,
    nav = 4 // Not really a PyPilot but simulated here
} tPyPilotMode;

typedef enum  {
    Standby = 0,
    Compass = 0x40,
    Wind = 0x100,
    Track = 0x180,
    NoDrift = 0x181
} tRaymarineMode;

typedef enum {
    TACKING_TO_PORT = -1,
    TACKING_NONE = 0,
    TACKING_TO_STARBOARD = 1
  } tTackDirection;

typedef enum  {
    TACK_NONE = 0,
    TACK_BEGIN = 1,
    TACK_WAITING = 2,
    TACK_TACKING = 3
  } tTackState;

  typedef enum {
    NO_DATA = 0,
    kNMEA2000 = 1,
    PYPILOT = 2
  } tDataOrigin;

  typedef enum {
    MOB_INACTIVE = 0,
    MOB_ACTIVE = 1,
    MOB_TEST = 2
  } tMOBState;

  typedef struct {
    unsigned long when;
    tDataOrigin origin;
    double value;
  } tDoubleData;

  typedef struct {
    unsigned long when;
    tDataOrigin origin;
    bool value;
  } tBoolData;

  typedef struct {
    unsigned long when;
    tDataOrigin origin;
    tPyPilotMode value;
  } tModeData;

typedef struct {
    unsigned long when;
    tDataOrigin origin;
    tN2kHeadingReference reference;
    double heading;
  } tHeadingData;
  
typedef struct {
    unsigned long when;
    tDataOrigin origin;
    tN2kRudderDirectionOrder direction;
    double command;
  } tRudderCommandData;

 typedef struct {
    unsigned long when;
    tDataOrigin origin;
    tTackState value;
  } tTackStateData;

typedef struct {
    unsigned long when;
    tDataOrigin origin;
    tTackDirection value;
  } tTackDirectionData;

  typedef struct {
    unsigned long when;
    tDataOrigin origin;
    double latitude;
    double longitude;
  } tCoordinateData;

  typedef struct {
    unsigned long when;
    tDataOrigin origin;

    tN2kHeadingReference bearingReference;

    double bearingOriginToDestination;
    double bearingPositionToDestination;
    double destinationLatitude;
    double destinationLongitude;
  } tNavigationData;

  typedef struct {
    unsigned long when;
    tDataOrigin origin;
    char mmsi[10];
    double latitude;
    double longitude;
    double cog;
    double sog;
    tMOBState state;
  } tMOBData;

#endif