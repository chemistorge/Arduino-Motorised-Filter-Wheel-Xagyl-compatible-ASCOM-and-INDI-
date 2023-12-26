# Arduino-Motorised-Filter-Wheel-Xagyl-compatible-ASCOM-and-INDI-
A 3d printable motorised filter wheel. The wheel is motorised with a 28BYJ-48 stepper with control provided by an Arduino Nano. The firmware is compatible with Xagyl Indi and ASCOM drivers.


I was in the market for a filter wheel, but didn't want to spend the money. I had recently got access to Fusion 360 and fancied desigining my own to print on my Ender 3 V2 printer.

In my research I found this 3D printable model for a filter wheel:https://www.thingiverse.com/thing:1292900

As I have created quite a few arduino projects I also decided to motorise the filter wheel and add capability for interfacing with INDI and ASCOM drivers. After abit of research
I found this GIThub: https://github.com/Blueshawk/Rayz-indi-wheel with an arduino sketch and protocol for the Xagyl brand of filter wheels.

While the existing code sort of worked I found it very buggy and with a high latency for serial commands. Also I could not get it working with ASCOM drivers. 

This GitHub contains the full 3d Printing Files and arduino firmware. The filter wheel is recongnised by both Indi and Ascom, functionally it is limited to changing filters and setting offsets. There is also in the design two buttons that can phyically change filters manually. Also a buzzer or led can be included to have local confirmation of which filter has just been selected.

There are two versions of the filter back plate (M42, and M48) design for either T2 connection or 48 mm common on coma correctors. The filter wheel introduces 21 mm back focus so is designed to replace a ZWO 21 mmm spacer.

The sensor used was an analog Hall Sensor from the 37 in 1 kit of sensors. The magnet used was 12mm x 2mm disc (fits into the raise part of the wheel), but any sized magnet <12 mm OD and <= 2 mm thickness will do. (if you would like help with using a digital Hall Sensor please contact me, the top part of the code has defines that can be changed to select analogue and digital hall sensors.

Main filter box is held together with M3 bolts and nuts (~25 mm). The circuitry lid uses M3 bolts that screw into plastic.

You may need to install the CH340 serial drivers for the arduino nano if using Mac or Windows (Linux supports natively). See here for help: https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all

Link to Discussion on Stargazer Lounge: https://stargazerslounge.com/topic/382864-diy-3d-printable-motorised-125-filter-wheel-ascom-and-indi/

Link to Discussion on Cloudy Nights: https://www.cloudynights.com/topic/788950-3d-printable-motorised-125-filter-wheel-ascom-and-indi-compatible/


Arduino code now updated to allow wheels with any number of filters.
