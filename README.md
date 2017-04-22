# Overview
In this repo I'm publishing a the results of a project we did a couple of years ago. We tried to read text over haptic feedback on the torso. We created a portable motor driver that is able to drive 32 vibration motors and could be connected to a smartphone or computer via bluetooth and serial console. We're not providing a detailed howto for building such a device. In fact it is quite easy to build one, it just requires a couple of hours of soldering. The images in the img folder provide some insight into the hardware.

# Results
Initially we intended to use each motor for another character in the alphabet. But this failed because we were not able to differentiate between the motors. We then used different combinations of motors to encode characters. After several training sessions (maybe 8 sessions with 20 minutes each) we were able to achieve speeds of up to 2 characters per second with an error rate of roughly 5%. As this was much lower than we hoped for, we stopped the project.

# Parts
* 1x ATMEGA16-16PU
* 4x ULN2803L
* 1x HC-06 Bluetooth Serial adapter
* Smaller parts are not listed here


# Participants
* Felix Heilmeyer https://github.com/heilerich
* Armin Feistenauer
* Sven Seeberg https://github.com/sven15
