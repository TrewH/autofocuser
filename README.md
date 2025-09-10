# autofocuser
A DIY solution to astrophotography telescope focusing. 

## Roadmap 
### Big Picture Architecture
Utilizies a Raspberry Pi running ROS2 (Humble on Ubuntu 22.04) and an INDI focuser driver. The ROS node commands the motor, and the INDI driver is a thin bridge to the ROS service/topics. 

### Hardware
Motor Driver: A4988
Microcontroller: Arduino Nano
ROS Host: Raspberry Pi
Stepper Motor: NEMA17

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
