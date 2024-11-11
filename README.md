### Autopilot 2000 - Bridge between Pypilot and NMEA 2000

The objective of this project is to make a Pypilot available as a NMEA2000 directly
so other devices recognize it as a full autopilot and may be used
to control it.

Specifically I want an ORCA Core to detect and accept it so it may be controlled
from the ORCA application. 

Here I simulate a Raymarine EVO device.

Added group processing :

    - We may set (by command) mode and locked heading
    - We may query (by requests) mode, locked heading, rudder, and vessel heading
    
The idea is NOT writing a full Autopilot application but just an interface
between PyPilot and NMEA2000 autopilots so it simulates a Raymarine EV-1 autopilot.

That means it should be  recognized by remotes, and other devices.

Nav mode is supported but relies in Orca sending PNG 129284 bearings. As far as I know
PyPilot does not support the routing automatically.

Of course we could use a different strategy but this one seems to work well.

You could use different controllers for the PyPilot depending of your hardware and software.

My installation is :

    - PyPilot is a TinyPilot
    - It connects to the ship network 
    - I have an controller with an M5Dial and this soft : https://github.com/fgorina/PyPilot-Controller, it works very well.

    - Of course it also works from OpenCPN.
    - All seems to be synchronized (if you enable wind auto in M5Dial it shows as that i Orca, etc).

    
References:



