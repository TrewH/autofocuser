# autofocuser
A DIY solution to astrophotography telescope focusing. 

## Roadmap 
### Big Picture Architecture
Utilizies a Raspberry Pi running ROS2 (Humble on Ubuntu 22.04) and an INDI focuser driver. The ROS node commands the motor, and the INDI driver is a thin bridge to the ROS service/topics. 

Ultrasonic distance sensor controls "home" position to move to on startup, as well as bounds for movement limits to prevent crashing at either end of the focuser tube. 

EEPROM memory utilized to maintain constant tracking of focuser position even in the event of power loss. 

### Hardware
Motor Driver: A4988
Microcontroller: Arduino Nano
ROS Host: Raspberry Pi
Stepper Motor: NEMA17
Ultrasonic Distance Sensor

### Focuser Positioning
Depending on camera and filter combination, home positions of the motor can be moved to for quickly nearing a prime focus

### Software
Topics: 
- /focuser/state
Services:
- /focuser/home -> Trigger
- /focuser/stop -> Trigger
Action:
/focuser/move_to

Config stored in config.yaml
