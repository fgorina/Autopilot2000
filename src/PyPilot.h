#ifndef PYPILOT_H
#define PYPILOT_H

#include <WiFi.h>
#include "NMEA2000.h"
#include "N2kTypes.h"
#include "State.h"
#include "PyTypes.h"


typedef struct _NetClient
{
    WiFiClient c = WiFiClient();
    unsigned long lastActivity = 0U;
} NetClient;


class PyPilot
{

protected:
    tNMEA2000 *nmea2000;
    
    NetClient pypClient = NetClient();

    String network = "Yamato";
    String passwd = "ailataN1991";

    IPAddress pyp_host = IPAddress(0, 0, 0, 0);
    int pyp_port = 0;

    TaskHandle_t loop_task;

    void pypilot_greet();
    bool pypilot_parse();
    boolean checkConnection();
    boolean startWiFi();
    tRaymarineMode pyPilot2RaymarineMode(tPyPilotMode mode);
    tPyPilotMode raymarine2PyPilotMode(tRaymarineMode mode);

    bool test = false;


public:
    tState *state {new tState()};

    bool isConnected();

 void set_host(IPAddress adr, int port);
 void set_nmea(tNMEA2000* pNemea);

    // Setting State
        void setRudderAngle(double angle, tDataOrigin from);
        void setHeading(double angle, tN2kHeadingReference reference, tDataOrigin from);
        void setVariation(double angle, tDataOrigin from);
        void setDeviation(double angle, tDataOrigin from);

        void setEngaged(bool eng, tDataOrigin from);
        void setRudderCommand(double angle, tN2kRudderDirectionOrder direction, tDataOrigin from);
        void setRaymarineMode(tRaymarineMode mode, tDataOrigin from);
        void setPypilotMode(tPyPilotMode mode, tDataOrigin from);
        void setCommandHeadingMagnetic(double heading, tDataOrigin from);
        void setCommandHeadingTrue(double heading, tDataOrigin from);
        void setTackState(tTackState tackSt, tDataOrigin from);
        void setTackDirection(tTackDirection direction, tDataOrigin from);

        void setServoVoltage(double voltage, tDataOrigin from);
        void setServoAmpHr(double amp, tDataOrigin from);
        void setServoVControllerTemp(double t, tDataOrigin from);
        void setServoPosition(double position, tDataOrigin from);



    // Updating PyPilot

    void pypilot_send_engage();
    void pypilot_send_disengage();
    void pypilot_send_command(double heading);
    void pypilot_send_rudder_command(double command);
    void pypilot_send_rudder_position(double pos);
    void pypilot_send_tack();
    void pypilot_send_cancel_tack();
    void pypilot_send_mode(tPyPilotMode mode);
    void pypilot_begin(String ssid, String pwd, IPAddress host = IPAddress(0,0, 0,0), int port = 0);
    void setKeepAlive();
    void disconnect_clients();
    void pypilot_one_pass();

   
      // Sending Information to NMEA2000

        void sendPilotMode(tNMEA2000 *NMEA2000);
        void sendVesselHeading(tNMEA2000 *NMEA2000);
        void sendRudder(tNMEA2000 *NMEA2000);
        void sendLockedHeading(tNMEA2000 *NMEA2000);
        void sendWindDatum(tNMEA2000 *NMEA2000);
};

#endif