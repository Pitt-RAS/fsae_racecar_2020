
# Firmware Execution Instructions #

These steps will explain how to upload and run code on the teeny using the Arduino IDE, as well as integrate it with the Jetson.

## Uploading Code to the Teensy ##

Prereqs:
	Arduino IDE (https://www.arduino.cc/en/Main/Software)
	Teensy Loader (https://www.pjrc.com/teensy/loader.html)

Steps:
1. Clone repo (git clone https://github.com/Pitt-RAS/fsae_racecar_2020.git)
2. cd /PATH/TO/REPO
3. Open the Arduino IDE and open /PATH/TO/REPO/src/fsae_firmware/magellan_controller/magellan_controller.ino. All the files in the directory should be fanned out in the 	file menu in the IDE. 	
4. Connect the teensy to the car via usb. Be sure the teensy loader program is running.
5. Click upload in the top left corner of the Arduino IDE. You may need to manually press the program button on the teensy (it depends what mood the teensy is in).

## Executing the Code ##

1. Clone repo (git clone https://github.com/Pitt-RAS/fsae_racecar_2020.git)
2. cd /PATH/TO/REPO
3. Open new terminal and run source rostarget.sh (do this in every new terminal you plan to interface with the Jetson with)
4. Start magellan stack on Jetson with ./robot.sh start
5. In order to verify that the code is running rostopic list and make sure you see multiple rostopics listed

## Misc ##

Every time you upload new code to the teensy, you must run ./robot.sh start AFTER plugging the teensy into the Jetson
