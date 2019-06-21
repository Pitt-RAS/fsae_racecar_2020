#include "config.h"
#include <ros.h>
#include "Rate.h"
#include "Robot.h"

ros::NodeHandle nh;
Rate loop_rate(MAIN_LOOP_HZ);

void setup() {
    nh.initNode();

    // Use full teensy ADC resolution
    analogReadResolution(13);

    Robot robot(nh);
    while (true) {
        robot.Update();
        nh.spinOnce();
        loop_rate.Sleep();
    }
}

void loop() {}
