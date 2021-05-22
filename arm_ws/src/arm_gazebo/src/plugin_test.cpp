
#include <iostream>
#include "ros/ros.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>

#include "log.h"

int main(int _argc, char **_argv) {
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the  arm topic
    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Quaternion>("~/arm/angle_cmd");

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    gazebo::msgs::Quaternion msg;

    while (true) {
        double angles[4];
        for (int i = 0; i < 4; i++) {
            std::cout << "Insert angle "<<  i + 1 << ' ';
            std::cin >> angles[i];
        }
        DebugMessage("Message published");

        // Set the velocity in the x-component
        gazebo::msgs::Set(&msg, ignition::math::Quaternion(angles[0], angles[1], angles[2], angles[3]));

        // Send the message
        pub->Publish(msg);
    }
    gazebo::client::shutdown();
    return 0;
}
