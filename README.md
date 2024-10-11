### Autopilot 2000 - Bridge between Pypilot and NMEA 2000

The objective of this project is to make a Pypilot available as a NMEA2000 directly
so other devices recognize it as a full autopilot and may be used
to control it.

Specifically I want an ORCA Core to detect and accept it so it may be controlled
from the ORCA application. 

If ORCA detects from the PGN's that are in EVO documentation it should work. 

Added group processing :

    - We may set (by command) mode and locked heading
    - We may query (by requests) mode, locked heading, rudder, and vessel heading
    
The idea is NOT writing a full Autopilot application but just an interface
between PyPilot and NMEA2000 autopilots so it simulates a Raymarine EV-1 autopilot.

That means it should be  recognized by remotes, and other devices.

TODO: Support keystrokes simulating a remote. First muyst see what ORCA sends and if it recognizes the device

