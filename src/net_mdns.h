#ifndef NET_MDNS_H
#define NET_MDNS_H

#include "PyPilot.h"
#include <ESPmDNS.h>
//   from command line you can discover services:
//   avahi-browse -ar

#ifdef __cplusplus
extern "C" {
#endif

 
  static bool mdns_up = false;

  bool mdns_begin() {
    if (!MDNS.begin("ESP32_Browser")) {
      return false;
    } else {
      mdns_up = true;
      return true;
    }
  }

  void mdns_end() {
    MDNS.end();
    mdns_up = false;
  }

  int mdns_query_svc(const char* service, const char* proto) {
    bool fail = false;
    if (!mdns_up) {
      fail = !mdns_begin();
    }
    if (!fail) {
      delay(50);
      return MDNS.queryService(service, proto);
    } else {
      return 0;
    }
  }

  void lookupPypilot(PyPilot* pypilot){
    /*
   M5Dial.Display.clear(BLACK);
    M5Dial.Display.setFont(&fonts::Orbitron_Light_24);
    M5Dial.Display.drawString("Searching", LV_HOR_RES_MAX / 2, LV_HOR_RES_MAX / 2);
  */

    Serial.println("Cercant pypilot");
      mdns_begin();
      int n = mdns_query_svc("pypilot", "tcp");
      if (n > 0) {
        pypilot->set_host(MDNS.IP(0), MDNS.port(0));
        
        Serial.print("Trobat Pyilot at ");
        Serial.print(MDNS.IP(0).toString());
        Serial.print(" port ");
        Serial.println(MDNS.port(0));

        
        //writePreferences();


      }else{
        pypilot->set_host(IPAddress(192, 168, 1, 148), 23322);
        
        Serial.print("No he trobat PyPilot, Default Pyilot at ");
        Serial.print("192.168.1.148");
        Serial.print(" port ");
        Serial.println(23322);
       // writePreferences();

      }
      mdns_end();
    
}

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
