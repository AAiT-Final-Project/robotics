//
// Created by Segni on 22/05/2021.
//

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "log.h"

void PrintJointStates(const sensor_msgs::JointState::ConstPtr& msg) {
    for (int i = 0; i < 4; i++) {
        double angleRad = msg->position[i];
        double angleDeg = angleRad * 180 / M_PI;
        DebugMessage("I heard: joint " + msg->name[i] + " has angle of " + std::to_string(angleDeg));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pub_joint_command_test");
    ros::NodeHandle n;

    ros::SubscribeOptions subscribeOptions =
            ros::SubscribeOptions::create<sensor_msgs::JointState>(
                    "joint_states", 1, PrintJointStates,
                    ros::VoidPtr(), n.getCallbackQueue());

    ros::Subscriber sub = n.subscribe(subscribeOptions);
    ros::spin();

    return 0;
}