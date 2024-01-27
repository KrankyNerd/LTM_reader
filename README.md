An arduino LTM interperter. Designed to run on Arduino Nano 33 IOT. Built using PlatformIO. 
I used it to read LTM data off of a Radiomaster TX16s running a lua script to convert telemetry data to LTM.

Telemtry to LTM lua script I am using on the TX16s:
https://github.com/stronnag/LTM-lua

Based off of this project:
https://github.com/sppnk/LTM-Telemetry-OLED

Other rescources I have collected for this project:
LTM definitions and format:
https://github.com/DzikuVx/ltm_telemetry_reader/blob/master/ltm-definition.md
Another LTM reader: its based off of this one but this one does not conform to the standard
https://github.com/sppnk/LTM-Telemetry-OLED
Discussion, link to docs:
https://www.rcgroups.com/forums/showthread.php?2848126-Turnigy-9x-easy-LTM-telemetry-OLED-mod
Inav LUA telem doc:
https://github.com/iNavFlight/inav/blob/master/docs/Telemetry.md
INAV LTM doc:
https://github.com/iNavFlight/inav/wiki/Lightweight-Telemetry-(LTM)
LTM specs:
https://quadmeup.com/ltm-light-telemetry-protocol/



Talk about Lua:
https://www.rcgroups.com/forums/showthread.php?3718549-First-attempt-at-using-the-UART-port-with-OpenTX-%28TX16S%29
http://rcdiy.ca/aa-telemetry-scripts-getting-started/
https://rcsettings.com/index.php/viewcategory/13-lua-scripts
Inav Lua, how to get the local variables (telem data):
https://github.com/iNavFlight/OpenTX-Telemetry-Widget/blob/master/src/SCRIPTS/TELEMETRY/iNav.lua
Inav telem data to LTM lua script (I USE THIS):
https://github.com/stronnag/LTM-lua

