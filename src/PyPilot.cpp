#include "PyPilot.h"
#include "NMEA2000.h"
#include "N2kTypes.h"
#include <lwip/sockets.h>
#include "net_mdns.h"
#include <N2kMessages.h>

void pypilot_loop(void *p);

// Configuration

 void PyPilot::set_host(IPAddress adr, int port){
    this->pyp_host = adr;
    this->pyp_port = port;
  }

void PyPilot::set_nmea(tNMEA2000* pNemea){
    nmea2000 = pNemea;
   }

   bool PyPilot::isConnected(){
    return pypClient.c.connected() && !test;
   }
// Sending to PyPilot

void PyPilot::pypilot_greet()
{
    if (isConnected())
    {
        pypClient.c.println("watch={\"ap.heading\":0.5}");
        pypClient.c.println("watch={\"ap.heading_command\":true}");
        pypClient.c.println("watch={\"ap.enabled\":true}");
        pypClient.c.println("watch={\"ap.mode\":true}");
        pypClient.c.println("watch={\"servo.amp_hours\":1}");
        pypClient.c.println("watch={\"servo.position\":0.0}");
        pypClient.c.println("watch={\"servo.controller_temp\":1}");
        pypClient.c.println("watch={\"servo.voltage\":1}");
        pypClient.c.println("watch={\"ap.tack.state\":0.5}");
        pypClient.c.println("watch={\"ap.tack.direction\":1}");
        pypClient.c.println("watch={\"rudder.angle\":0}");
        //PACO pypClient.c.flush();
    }
}

void PyPilot::pypilot_send_engage()
{
    if (isConnected())
    {
        pypClient.c.println("ap.enabled=true");
        //PACO pypClient.c.flush();
        pypilot_send_command(state->heading.heading);
        pypilot_send_mode(state->mode.value);
    }
    
}

void PyPilot::pypilot_send_disengage()
{
    if (isConnected())
    {
        pypClient.c.println("ap.enabled=false");
        //PACO pypClient.c.flush();
    }
}

void PyPilot::pypilot_send_command(double heading)
{
    if (isConnected())
    {
        pypClient.c.print("ap.heading_command=");
        pypClient.c.println(heading, 1);
        //PACO pypClient.c.flush();
    }
}

void PyPilot::pypilot_send_rudder_command(double command)
{

    if (isConnected())
    {
        // Serial.print("Sending rudderCommand "); USBSerial.println(String(command, 2));
        pypClient.c.print("servo.command=");
        pypClient.c.println(command, 2);
        //PACO pypClient.c.flush();
    }
}

void PyPilot::pypilot_send_rudder_position(double pos)
{
    if (isConnected())
    {
        pypClient.c.print("servo.position_command=");
        pypClient.c.println(pos, 2);
        //PACO pypClient.c.flush();
    }
}

void PyPilot::pypilot_send_tack()
{
    Serial.print("Tacking to ");
    tTackDirection direction = state->tackDirection.value;
    Serial.println(direction);
    if (isConnected() && direction != tTackDirection::TACKING_NONE)
    {
        
        pypClient.c.print("ap.tack.direction=\"");
        pypClient.c.print(direction == tTackDirection::TACKING_TO_STARBOARD ? "starboard" : "port");
        pypClient.c.println("\"");
        pypClient.c.println("ap.tack.state=\"begin\"");
        //PACO pypClient.c.flush();
    }
}

void PyPilot::pypilot_send_cancel_tack()
{
    if (isConnected())
    {
        pypClient.c.println("ap.tack.state=\"none\"");
        //PACO pypClient.c.flush();
    }
}

void PyPilot::pypilot_send_mode(tPyPilotMode mode)
{

    Serial.print("Sending mode ");
    Serial.println(mode);
    if (isConnected())
    {
        switch(mode){
            case tPyPilotMode::compass: 
                 pypClient.c.println("ap.mode=\"compass\"");
                 break;
           case tPyPilotMode::gps: 
                 pypClient.c.println("ap.mode=\"gps\"");
                 break;
          case tPyPilotMode::wind: 
                 pypClient.c.println("ap.mode=\"wind\"");
                 break;
          case tPyPilotMode::trueWind: 
                 pypClient.c.println("ap.mode=\"true wind\"");
                 break;
           
           case tPyPilotMode::nav:
                // Send compass. Orca or Nav Computer sends headings that are converted
                // to magnetic headings. Seems to work best

                 pypClient.c.println("ap.mode=\"gps\"");
                 break;

        }
        //PACO pypClient.c.flush();
    }
}


 bool PyPilot::pypilot_parse() {
    bool found = false;
    String dataFeed = pypClient.c.readStringUntil('\n');
    // ap.heading=164.798
    // ap.heading_command=220.0000
    // ap.enabled=false
    // ap.mode="compass"
    if (dataFeed.length() > 0) {
      found = true;
      if (dataFeed.startsWith("ap.heading=")) {
        double oldHeading = state->heading.heading;
        double newHeading = strtof(dataFeed.substring(strlen("ap.heading="), dataFeed.length()).c_str(), NULL);
        setHeading(newHeading, tN2kHeadingReference::N2khr_magnetic, tDataOrigin::PYPILOT);
        
      } else if (dataFeed.startsWith("ap.heading_command=")) {
        double oldCommand = state->headingCommandMagnetic.value;
        double newCommand = strtof(dataFeed.substring(strlen("ap.heading_command="), dataFeed.length()).c_str(), NULL);
        if(state->mode.value != tPyPilotMode::nav){
            setCommandHeadingMagnetic(newCommand, tDataOrigin::PYPILOT);
        }else{
            setCommandHeadingTrue(newCommand, tDataOrigin::PYPILOT);
        }

      } else if (dataFeed.startsWith("ap.enabled=true")) {

        bool oldEngaged = state->engaged.value;
        setEngaged(true, tDataOrigin::PYPILOT);

      } else if (dataFeed.startsWith("ap.enabled=false")) {
        bool oldEngaged = state->engaged.value;
        setEngaged(false, tDataOrigin::PYPILOT);

      } else if (dataFeed.startsWith("ap.tack.state=\"")) {
        tTackState oldTackState = state->tackState.value;
        String newTackState = dataFeed.substring(strlen("ap.tack.state=\""), dataFeed.length() - 1);

        if(newTackState == "begin"){
          setTackState(tTackState::TACK_BEGIN, tDataOrigin::PYPILOT);
        }else if (newTackState == "waiting"){
          setTackState(tTackState::TACK_WAITING, tDataOrigin::PYPILOT);
        }else if (newTackState == "tacking"){
          setTackState(tTackState::TACK_TACKING, tDataOrigin::PYPILOT);
        }else{
           setTackState(tTackState::TACK_NONE, tDataOrigin::PYPILOT);
        }
        
      }else if (dataFeed.startsWith("ap.tack.direction=\"")) {
        tTackDirection oldDirection = state->tackDirection.value;
   
        String direction = dataFeed.substring(strlen("ap.tack.direction=\""), dataFeed.length() - 1);
        
        if(direction == "starboard"){
            setTackDirection(tTackDirection::TACKING_TO_STARBOARD, tDataOrigin::PYPILOT);
        }else {
            setTackDirection(tTackDirection::TACKING_TO_PORT, tDataOrigin::PYPILOT);    
        }

      }else if (dataFeed.startsWith("ap.mode=\"")) {
        tPyPilotMode oldMode = state->mode.value;

        String mode = dataFeed.substring(strlen("ap.mode=\""), dataFeed.length() - 1);
        Serial.print("Received "); Serial.println(mode);

        if(oldMode != tPyPilotMode::nav){
        if (mode == "gps") {
          setPypilotMode(tPyPilotMode::gps, tDataOrigin::PYPILOT);
        } else if (mode == "wind") {
          setPypilotMode(tPyPilotMode::wind, tDataOrigin::PYPILOT);
        } else if (mode == "compass") {
         setPypilotMode(tPyPilotMode::compass, tDataOrigin::PYPILOT);
        } else if (mode == "true wind") {
          setPypilotMode(tPyPilotMode::trueWind, tDataOrigin::PYPILOT);
        }
        }
        
      } else if (dataFeed.startsWith("servo.voltage=")) {
        double oldVoltage = state->servoVoltage.value;
        double voltage =   strtof(dataFeed.substring(strlen("servo.voltage="), dataFeed.length()).c_str(), NULL);
        setServoVoltage(voltage, tDataOrigin::PYPILOT);
        
      } else if (dataFeed.startsWith("servo.amp_hours=")) {
        double ampHr = strtof(dataFeed.substring(strlen("servo.amp_hours="), dataFeed.length()).c_str(), NULL);
        setServoAmpHr(ampHr, tDataOrigin::PYPILOT);
        
   
      } else if (dataFeed.startsWith("servo.controller_temp=")) {
        double temp =
          strtof(dataFeed.substring(strlen("servo.controller_temp="), dataFeed.length()).c_str(), NULL);
         setServoVControllerTemp(temp, tDataOrigin::PYPILOT);

      } else if (dataFeed.startsWith("servo.position=")) {
        double oldServoPos = state->servoPosition.value;
        double newServoPos = strtof(dataFeed.substring(strlen("servo.position="), dataFeed.length()).c_str(), NULL);
        
        setServoPosition(newServoPos, tDataOrigin::PYPILOT);
      
      }else if (dataFeed.startsWith("rudder.angle=")) {
        double oldRudderAngle = state->rudderAngle.value;
        double newRudderAngle  =  strtof(dataFeed.substring(strlen("rudder.angle="), dataFeed.length()).c_str(), NULL);
        setRudderAngle(newRudderAngle, tDataOrigin::PYPILOT);
      }else {
        found = false;
      }      
    }
    return found;
  }

// PyPilot Connection

boolean PyPilot::checkConnection()
{                  // Check wifi connection.
    int count = 0; // count.
    while (count < 30)
    { // If you fail to connect to wifi within 30*350ms (10.5s), return false; otherwise return true.
        if (WiFi.status() == WL_CONNECTED)
        {
            return true;
        }
        delay(350);
        count++;
    }
    return false;
}

boolean PyPilot::startWiFi()
{ // Check whether there is wifi configuration information storage, if there is return 1, if no return 0.

    /*
      M5Dial.Display.clear(BLACK);
      M5Dial.Display.setFont(&fonts::Orbitron_Light_24);
      M5Dial.Display.drawString("Connecting to ", LV_HOR_RES_MAX / 2, LV_HOR_RES_MAX / 2 - 16);
      M5Dial.Display.setFont(&fonts::Orbitron_Light_32);
      M5Dial.Display.drawString(wifi_ssid, LV_HOR_RES_MAX / 2, LV_HOR_RES_MAX / 2 + 16);
    */
    // WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.mode(WIFI_STA);
    Serial.print(network);
    Serial.print(" ");
    Serial.println(passwd);
    WiFi.begin(network.c_str(), passwd.c_str());
    if (checkConnection())
    {
        Serial.print("Connected to ");
        Serial.print(network);
        Serial.print(" IP ");
        Serial.println(WiFi.localIP());

        // Should enable another LED

        //lookupPypilot(this);

        /*
        M5Dial.Display.clear(BLACK);
        M5Dial.Display.setFont(&fonts::Orbitron_Light_24);
        M5Dial.Display.drawString("Connecting to ", LV_HOR_RES_MAX / 2, LV_HOR_RES_MAX / 2 - 16);

        M5Dial.Display.drawString("PyPilot", LV_HOR_RES_MAX / 2, LV_HOR_RES_MAX / 2 + 16);
        M5Dial.Display.drawString(pypilot_tcp_host.toString(), LV_HOR_RES_MAX / 2, LV_HOR_RES_MAX / 2 + 44);
        */
        delay(1000);

        // M5Dial.Display.setFont(&fonts::Orbitron_Light_32);

        return true;
    }
    return false;
}
void PyPilot::pypilot_begin(String ssid, String pwd, IPAddress host, int port)
{

    this->pyp_host = host;
    this->pyp_port = port;
    this->network = ssid;
    this->passwd = pwd;

    Serial.print("PyPilot Host ");
    Serial.print(pyp_host);
    Serial.print(" : ");
    Serial.println(pyp_port);

    // Configure WiFi

    xTaskCreatePinnedToCore(pypilot_loop, "PyPilot Loop", 4000, this, 1, &this->loop_task, 0);
}

void PyPilot::setKeepAlive()
{
    int flags = 1;
    pypClient.c.setSocketOption(SOL_SOCKET, SO_KEEPALIVE, (const void *)&flags, sizeof(flags));
    flags = 10;
    pypClient.c.setSocketOption(IPPROTO_TCP, TCP_KEEPIDLE, (const void *)&flags, sizeof(flags));
    flags = 5;
    pypClient.c.setSocketOption(IPPROTO_TCP, TCP_KEEPCNT, (const void *)&flags, sizeof(flags));
    flags = 5;
    pypClient.c.setSocketOption(IPPROTO_TCP, TCP_KEEPINTVL, (const void *)&flags, sizeof(flags));
}

void PyPilot::disconnect_clients()
{
    if (isConnected())
    {
        pypClient.c.stop();
    }
}

void PyPilot::pypilot_one_pass()
{
    if (WiFi.status() != WL_CONNECTED) // Connect to WiFI
    {

        if (startWiFi())
        {
            Serial.println("Connected to WiFi");
        }
        else
        {
            Serial.println("Unable to connect to WiFi!!!");
        }
    }
    else // Already connected to Wifi.
    {
        if (pypClient.c.connected()) // Process possible data
        {
            if (pypClient.c.available() > 12)
            {
                bool found = pypilot_parse();
                if (found)
                {
                    pypClient.lastActivity = millis();
                }
            }
        }
        else // Connect to PyPilot and subscribne
        {
            setKeepAlive();
            lookupPypilot(this);
            if (pypClient.c.connect(pyp_host, pyp_port, 200))
            {
                Serial.println("Reconnected to pypilot");
                pypilot_greet();
            }
            else
            {
                Serial.println("Failed to connect to pypilot in reconnect");
            }
        }
    }
}

 
/// Sending Data to PyPilot
tRaymarineMode PyPilot::pyPilot2RaymarineMode(tPyPilotMode rmode)
{
    if(!state->engaged.value){
        return tRaymarineMode::Standby;
    }
   switch (rmode){
    case tPyPilotMode::compass:
        return tRaymarineMode::Compass;         
        break;
    case tPyPilotMode::gps:
        return tRaymarineMode::NoDrift; // ???
        break;
   case tPyPilotMode::wind:
        return tRaymarineMode::Wind;         
        break;
   case tPyPilotMode::trueWind:
        return tRaymarineMode::NoDrift; // ???        
        break;

    case tPyPilotMode::nav:
        return tRaymarineMode::Track;
        break;

    default:
        return tRaymarineMode::Standby;

   }
}

tPyPilotMode PyPilot::raymarine2PyPilotMode(tRaymarineMode rmode)
{
    switch (rmode)
    {
    case tRaymarineMode::Standby:
        return state->mode.value;

    case tRaymarineMode::Compass:
        return tPyPilotMode::compass;

    case tRaymarineMode::Wind:
        return tPyPilotMode::wind;

    case tRaymarineMode::Track:
        return tPyPilotMode::nav;

    case tRaymarineMode::NoDrift:
        return tPyPilotMode::gps;

    default:
        Serial.print("Error en Raymarine Pilot Mode : ");
        Serial.println(rmode);
        return tPyPilotMode::compass;
    }
}

void PyPilot::setRudderAngle(double angle, tDataOrigin from)
{
    state->rudderAngle.value = angle;
    state->rudderAngle.origin = from;
    state->rudderAngle.when = millis();

 
    if (from == tDataOrigin::PYPILOT)
    {
        sendRudder(nmea2000);
    } 
}

void PyPilot::setEngaged(bool eng, tDataOrigin from)
{
    state->engaged.value = eng;
    state->engaged.origin = from;
    state->engaged.when = millis();

    if (from != tDataOrigin::PYPILOT)
    {
        if (eng){
            pypilot_send_engage();
        }else{
            pypilot_send_disengage();
        }     
    }
    else
    {
        sendPilotMode(nmea2000);
    }
}
void PyPilot::setRudderCommand(double angle, tN2kRudderDirectionOrder direction, tDataOrigin from)
{
    state->rudderCommand.command = angle;
    state->rudderCommand.direction = direction;
    state->rudderCommand.origin = from;
    state->rudderCommand.when = millis();

    if (from != tDataOrigin::PYPILOT)
    {
        // Send to PyPilot
    }
    else
    {
        // Send to NMEA???
    }
}

void PyPilot::setRaymarineMode(tRaymarineMode rmode, tDataOrigin from)
{

    if (rmode == tRaymarineMode::Standby)
    {
       
        if (state->mode.value == tPyPilotMode::nav){  // nav mode does not really exist
            state->mode.value = tPyPilotMode::gps;
            state->mode.origin = tDataOrigin::kNMEA2000;
            state->mode.when = millis();
        }
        setEngaged(false, from);
        Serial.println("Received Standby. from NMEA Setting Engaged = false");
    }
    else
    {
        unsigned long t = millis();
        state->mode.value = raymarine2PyPilotMode(rmode);
        state->mode.origin = from;
        state->mode.when = millis();

        setEngaged(true, from); // Already sends the command

        Serial.print("Received mode ");
        Serial.print(rmode);
        Serial.print(" from NMEA Seting mode to ");
        Serial.print(state->mode.value);
        Serial.println(" and engaged = true");
    }
}
void PyPilot::setPypilotMode(tPyPilotMode rmode, tDataOrigin from)
{

    state->mode.value = rmode;
    state->mode.origin = from;
    state->mode.when = millis();

    if (from != tDataOrigin::PYPILOT)
    {
        pypilot_send_mode(state->mode.value);
    }
    else
    {
        if (state->engaged.value)
        { 
            sendPilotMode(nmea2000);
        }
    }
    Serial.print("Received mode ");
    Serial.print(rmode);
    Serial.println(" from PyPilot ");

 
}
void PyPilot::setTackState(tTackState tackSt, tDataOrigin from)
{
    state->tackState.value = tackSt;
    state->tackState.origin = from;
    state->tackState.when = millis();

    if (from == tDataOrigin::kNMEA2000)
    {
       if(tackSt == tTackState::TACK_NONE){
        pypilot_send_cancel_tack();
       }else if (tackSt == tTackState::TACK_BEGIN){
        pypilot_send_tack();
       }
    }
    else
    {
        // Send to NMEA
    }
}
void PyPilot::setTackDirection(tTackDirection direction, tDataOrigin from)
{
    state->tackDirection.value = direction;
    state->tackState.origin = from;
    state->tackState.when = millis();

    // No need , just store data and setTackState manages everything
    if (from == tDataOrigin::kNMEA2000)
    {
        // Send to PyPilot
    }
    else
    {
        // Send to NMEA
    }
}
// Heading is just info. May not change Heading!!!

void PyPilot::setHeading(double angle, tN2kHeadingReference ref, tDataOrigin from)
{
    if(from != tDataOrigin::PYPILOT){
        state->heading.heading = angle * 180.0 / PI;
    }else{
        state->heading.heading = angle;
    }
    
    state->heading.reference = ref;
    state->heading.origin = from;
    state->heading.when = millis();

     //Serial.print("Receiving Heading from NMEA : "); Serial.print(state->heading.heading);
     //Serial.print(" Origin: "); Serial.print(from);Serial.print(" Ref: "); Serial.println(ref);

   
}

void PyPilot::setVariation(double angle, tDataOrigin from)
{
    state->variation.value = angle / PI * 180.0;
    state->variation.origin = from;
    state->variation.when = millis();
}

void PyPilot::setDeviation(double angle, tDataOrigin from)
{
    state->deviation.value = angle / PI * 180.0;
    state->deviation.origin = from;
    state->deviation.when = millis();
}
// Servo Info
void PyPilot::setServoVoltage(double voltage, tDataOrigin from)
{
    state->servoVoltage.value = voltage;
    state->servoVoltage.origin = from;
    state->servoVoltage.when = millis();
}
void PyPilot::setServoAmpHr(double amp, tDataOrigin from)
{
    state->servoAmpHr.value = amp;
    state->servoAmpHr.origin = from;
    state->servoAmpHr.when = millis();
}
void PyPilot::setServoVControllerTemp(double t, tDataOrigin from)
{
    state->servoControllerTemp.value = t;
    state->servoControllerTemp.origin = from;
    state->servoControllerTemp.when = millis();
}
void PyPilot::setServoPosition(double position, tDataOrigin from)
{
    state->servoPosition.value = position;
    state->servoPosition.origin = from;
    state->servoPosition.when = millis();
}

// Here we set the command to pypilot
void PyPilot::setCommandHeadingTrue(double heading, tDataOrigin from)
{
  
    if (from != tDataOrigin::PYPILOT)
    {

        state->headingCommandTrue.value = heading / PI * 180.0;
        state->headingCommandTrue.origin = from;
        state->headingCommandTrue.when = millis();

        state->headingCommandMagnetic.value =  heading / PI * 180.0  - state->variation.value;
        state->headingCommandMagnetic.origin = from;
        state->headingCommandMagnetic.when = millis();

        /*
        Serial.print("Receiving command heading from NMEA to (true) ");
        Serial.print(state->headingCommandTrue.value);
        Serial.print(" Magnetic ");
        Serial.println(state->headingCommandMagnetic.value);  // We translate according variation. 
        */

        if (state->mode.value == tPyPilotMode::nav){
            pypilot_send_command(state->headingCommandTrue.value);
        }else{
            pypilot_send_command(state->headingCommandMagnetic.value);
        }
        
        // Send to PyPilot
    }
    else
    {

        state->headingCommandTrue.value = heading;
        state->headingCommandTrue.origin = from;
        state->headingCommandTrue.when = millis();

        state->headingCommandMagnetic.value = heading  - state->variation.value;
        state->headingCommandMagnetic.origin = from;
        state->headingCommandMagnetic.when = millis();

        /*
        Serial.print("Receiving command heading from PyPilot to (true) ");
        Serial.println(heading);
        */
        sendLockedHeading(nmea2000);
        sendHeadingTrackControl(nmea2000);
    }
}

void PyPilot::setCommandHeadingMagnetic(double heading, tDataOrigin from)
{
    
    
    if (from != tDataOrigin::PYPILOT)
    {
        state->headingCommandMagnetic.value = heading / PI * 180.0;
        state->headingCommandMagnetic.origin = from;
        state->headingCommandMagnetic.when = millis();

        state->headingCommandTrue.value = heading / PI * 180.0 + state->variation.value;
        state->headingCommandTrue.origin = from;
        state->headingCommandTrue.when = millis();

        Serial.print("Receiving command heading from NMEA to (magnetic) ");
        Serial.println(state->headingCommandMagnetic.value);
        pypilot_send_command(state->headingCommandMagnetic.value);
    }
    else
    {
        state->headingCommandMagnetic.value = heading;
        state->headingCommandMagnetic.origin = from;
        state->headingCommandMagnetic.when = millis();

        state->headingCommandTrue.value = heading + state->variation.value;
        state->headingCommandTrue.origin = from;
        state->headingCommandTrue.when = millis();

        Serial.print("Receiving command heading from PyPilot to (magnetic) ");
        Serial.println(heading);
        sendLockedHeading(nmea2000);
        sendHeadingTrackControl(nmea2000);
    }
}

  // NMEA 2000
void PyPilot::sendPilotMode(tNMEA2000 *NMEA2000)
{

    tN2kMsg N2kMsg;
    N2kMsg.SetPGN(65379L);
    N2kMsg.AddByte(0x3B);
    N2kMsg.AddByte(0x47);

    uint16_t rmode = pyPilot2RaymarineMode(state->mode.value);
    //Serial.print("Sending mode of ");
    //Serial.println(rmode);

    N2kMsg.Add2ByteUInt(rmode); // mode
    N2kMsg.Add2ByteUInt(0L);    // submode
    N2kMsg.AddByte(0);          // Pilot Mode Data
    N2kMsg.AddByte(0);          // Reserved

    NMEA2000->SendMsg(N2kMsg);
}

void PyPilot::sendVesselHeading(tNMEA2000 *NMEA2000)
{
    Serial.print("Sending vessel heading of ");
    Serial.println(state->heading.heading);
    tN2kMsg N2kMsg;
    N2kMsg.SetPGN(127250);
    double radVesselHeading = state->heading.heading / 180.0 * 3.141592;

    N2kMsg.Add2ByteUDouble(radVesselHeading, 0.0001);
    N2kMsg.Add2ByteUDouble(-1E9, 0.0001);
    N2kMsg.Add2ByteUDouble(-1E9, 0.0001);
    N2kMsg.AddByte(state->heading.reference); // Magnetioc (true is 0)

    NMEA2000->SendMsg(N2kMsg);
}
void PyPilot::sendHeadingTrackControl(tNMEA2000 *NMEA2000)
{
   
    tN2kMsg N2kMsg;
    double radVesselHeading = state->heading.heading / 180.0 * 3.141592;
    double radHeadingToSteer = state->headingCommandMagnetic.value / 180.0 * 3.141592;


    SetN2kPGN127237(
        N2kMsg,
        N2kOnOff_Unavailable,     // RudderLimitExceeded
        N2kOnOff_Unavailable,     // OffHeadingLimitExceeded
        N2kOnOff_Unavailable,     // OffTrackLimitExceeded
        N2kOnOff_Unavailable,     // Override
        N2kSM_Unavailable,            // SteeringMode
        N2kTM_Unavailable,            // TurnMode
        N2khr_magnetic,                      // HeadingReference (True/Magnetic)
        N2kRDO_Unavailable,           // CommandedRudderDirection
        N2kDoubleNA,              // CommandedRudderAngle
        radHeadingToSteer,           // HeadingToSteerCourse (radians)
        N2kDoubleNA,              // Track
        N2kDoubleNA,              // RudderLimit
        N2kDoubleNA,              // OffHeadingLimit
        N2kDoubleNA,              // RadiusOfTurnOrder
        N2kDoubleNA,              // RateOfTurnOrder
        N2kDoubleNA,              // OffTrackLimit
        radVesselHeading             // VesselHeading (radians)
    );

    NMEA2000->SendMsg(N2kMsg);
}
void PyPilot::sendRudder(tNMEA2000 *NMEA2000)
{
    tN2kMsg N2kMsg;
    N2kMsg.SetPGN(127245);
    //Serial.print("Rudder Angle "); Serial.println(state->rudderAngle.value );

    double radRudderAngle = state->rudderAngle.value / 180.0 * 3.141592;
    N2kMsg.AddByte(1); // Instance. Only one rudder  
    N2kMsg.AddByte(0);                    // No rudder order
    N2kMsg.Add2ByteDouble(-1E9, 0.0001); // Rudder Order
    N2kMsg.Add2ByteDouble(-radRudderAngle, 0.0001); // Changed direction so direction is ok
    N2kMsg.Add2ByteUInt(0); // Reserved

    NMEA2000->SendMsg(N2kMsg);
}

void PyPilot::sendLockedHeading(tNMEA2000 *NMEA2000)
{
    //Serial.print("Sending locked heading of ");
    //Serial.println(state->headingCommandMagnetic.value);
    //Serial.println("------------------------------------------------------------------------------");
    tN2kMsg N2kMsg;
    N2kMsg.SetPGN(65360);
    double radLockedHeadingTrue = state->headingCommandTrue.value / 180.0 * 3.141592;
    double radLockedHeadingMagnetic = state->headingCommandMagnetic.value / 180.0 * 3.141592;

    N2kMsg.AddByte(0x3B); // Raymarine, Marine
    N2kMsg.AddByte(0x47);

    N2kMsg.AddByte(1); // SID
    N2kMsg.Add2ByteDouble(radLockedHeadingTrue, 0.0001);
    N2kMsg.Add2ByteDouble(radLockedHeadingMagnetic, 0.0001);

    N2kMsg.AddByte(0); // Reserved

    NMEA2000->SendMsg(N2kMsg);
}
void PyPilot::sendWindDatum(tNMEA2000 *NMEA2000){

   // Serial.print("Sending datum  heading of ");
   // Serial.println(state->headingCommandMagnetic.value);
    tN2kMsg N2kMsg;
    N2kMsg.SetPGN(65345);
    double radLockedHeadingTrue = state->headingCommandMagnetic.value / 180.0 * 3.141592;
    double radLockedHeadingMagnetic = state->heading.heading / 180.0 * 3.141592;

    N2kMsg.AddByte(0x3B); // Raymarine, Marine
    N2kMsg.AddByte(0x47);

    N2kMsg.Add2ByteDouble(radLockedHeadingTrue, 0.0001);        // Wind Datum
    N2kMsg.Add2ByteDouble(radLockedHeadingMagnetic, 0.0001);    // Rolling Average Wind Angle

    N2kMsg.AddByte(0); // Reserved

    NMEA2000->SendMsg(N2kMsg);


}   

// Task function!!!

void pypilot_loop(void *p)
{
    PyPilot *py = static_cast<PyPilot *>(p);
    while (true)
    {
        py->pypilot_one_pass();
        // Perhaps un vTaskDelay??

        vTaskDelay(10);
    }
}
