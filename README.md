# fsae_racecar_2020
Code for interfacing with Jetson and connecting remote ROS nodes

These steps will explain how to deploy and execute new code on the teensy, and connect your ROS node to the Jetson.

**To upload new code to the teensy:**

Prereqs: 

Arduino IDE (https://www.arduino.cc/en/Main/Software) 

Teensy Loader (https://www.pjrc.com/teensy/loader.html)

1. Clone repo (git clone https://github.com/Pitt-RAS/fsae_racecar_2020.git)
2. cd /PATH/TO/REPO
3. git checkout jank (switch to jank branch where Noah's poorly updated firmware is kept)
4. Open Arduino IDE and open /PATH/TO/REPO/src/fsae_firmware/magellan_controller/magellan_controller.ino	
	You will notice that all the files in the magellan_controller directory open in separate tabs in the IDE. If you did not notice this, you did something wrong or Noah's readme sucks.
5. Connect teensy via usb. Be sure the teensy loader is running.
6. Click upload in the top left corner of the Arduino IDE. You may need to manually press the program button on the teensy (it depends what mood the teensy is in).


**To connect to the Jetson and start the ros stuff:**
1. Clone repo 
   `git clone https://github.com/Pitt-RAS/fsae_racecar_2020.git`
2. `cd /PATH/TO/REPO`
3. `git checkout jank` (switch to jank branch where Noah's poorly updated firmware is kept)
4. Run `roscore`
5. Open new terminal and run (do this in every new terminal you plan to interface with the Jetson with. Or add it to your .bashrc if you're lazy)
	 `source rostarget.sh`
	 `./rostarget.sh`
6. Start magellan stack on Jetson with 
   `./robot.sh start`
7. Once that does it's thing, you should be able to see the Magellan topics with `rostopic list`


Misc:
Every time you upload new code to the teensy, you must run ./robot.sh start AFTER plugging the teensy into the Jetson (yes, it's annoying, but I'm no Lobos)
