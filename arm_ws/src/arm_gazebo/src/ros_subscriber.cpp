
#ifndef _ROS_SUBSCRIBER_PLUGIN_HH_
#define _ROS_SUBSCRIBER_PLUGIN_HH_

//  Gazebo api
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Quaternion.h"
#include "log.h"

namespace gazebo
{
/// \brief A plugin to control a SUBSCRIBER sensor.
    class JointRosSubscriberPlugin : public ModelPlugin {

    private:

        /// \brief Pointer to the model.
        physics::ModelPtr model;

        /// \brief List of joints.
        physics::Joint_V jointList;

        /// \brief A PID controller for the joint.
        common::PID pid;

        /// \brief joint controller
        physics::JointControllerPtr jointController;

        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;

        /// \brief list of angles
        double angles[4];

        /// \brief ROS helper function that processes messages
        void QueueThread() {
            static const double timeout = 0.01;
            while (this->rosNode->ok()) {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    public:
        /// \brief Constructor
        JointRosSubscriberPlugin(){ }

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            printf("Joint ros subscriber \n");
            // Safety check
            if (_model->GetJointCount() == 0)
            {
                std::cerr << "Invalid joint count, SUBSCRIBER plugin not loaded\n";
                return;
            }

            // Store the model pointer for convenience.
            this->model = _model;

            // Get list of joints
            this->jointList = _model->GetJoints();

            // Setup a PID  .
            this->pid = common::PID(20.1, 10.01, 10.03);

            this->jointController = this->model->GetJointController();

            // Apply the PID to the joints.
            for (auto const &joint : jointList) {
                jointController->SetPositionPID(joint->GetScopedName(), this->pid);
            }

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized()) {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                          ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
                    ros::SubscribeOptions::create<geometry_msgs::Quaternion>(
                            "/arm/angle_cmd",
                            1,
                            boost::bind(&JointRosSubscriberPlugin::OnRosMsg, this, _1),
                            ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
                    std::thread(std::bind(&JointRosSubscriberPlugin::QueueThread, this));
        } // Load

        /// \brief Set the position of the SUBSCRIBER
        /// \param[in] iterating over list of joints it sets target position
        void SetJointAngle() {
            for (int i = 0; i < 4; i++) {
                double rad = M_PI * angles[i] / 180;
                jointController->SetPositionTarget(jointList[i]->GetScopedName(), rad);
            }
        }

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the position
        /// of the SUBSCRIBER.
        void OnRosMsg(const geometry_msgs::QuaternionConstPtr &_msg) {
            DebugMessage("Message received");

            this->angles[0] = _msg->x; // represents the bottom angle value
            this->angles[1] = _msg->y;
            this->angles[2] = _msg->z;
            this->angles[3] = _msg->w; // this has value of the top angle
            
            SetJointAngle();
        }
    };

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(JointRosSubscriberPlugin)
}
#endif