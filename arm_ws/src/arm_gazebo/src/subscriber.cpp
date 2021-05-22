
#ifndef JOINT_SUBSCRIBER_PLUGIN_HH_
#define JOINT_SUBSCRIBER_PLUGIN_HH_

#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "ros/ros.h"

#include "log.h"

namespace gazebo
{
    /// \brief A plugin to control joints.
    class JointSubscriberPlugin : public ModelPlugin {

    private:
        /// \brief A node used for transport
        transport::NodePtr node;

        /// \brief A subscriber to a named topic.
        transport::SubscriberPtr sub;

        /// \brief Pointer to the model.
        physics::ModelPtr model;

        /// \brief List of joints.
        physics::Joint_V jointList;

        /// \brief joint controller
        physics::JointControllerPtr jointController;

        /// \brief A PID controller for the joint.
        common::PID pid;


        /// \brief Handle incoming message
        /// \param[in] _msg Repurpose a vector3 message. This function will
        /// only use the x component.
        void OnMsg(ConstQuaternionPtr &_msg) {
            DebugMessage("Message received");
            double angles[4];
            angles[0] = _msg->y();
            angles[1] = _msg->z();
            angles[2] = _msg->w();
            angles[3] = _msg->x();

            int index = 0;
            for (auto joint : jointList) {
                float rad = M_PI * angles[index++] / 180;
                SetJointAngle(joint->GetScopedName(), rad);
            }
        }
    public:
        JointSubscriberPlugin() {}
        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            // Safety check
            printf("subscriber started\n");

            if (_model->GetJointCount() == 0) {
                std::cerr << "Invalid joint count, Arm_gazebo plugin not loaded\n";
                return;
            }

            // Store the model pointer for convenience.
            this->model = _model;
            this->jointController = this->model->GetJointController();
            // Get the first joint. We are making an assumption about the model
            // having one joint that is the rotational joint.
            this->jointList = _model->GetJoints();

            // Setup a P-controller, with a gain of 0.1.
            this->pid = common::PID(0.1, 0.01, 0.03);

            // Apply the P-controller to the joint.
            for (auto const &joint : jointList) {
                jointController->SetPositionPID(joint->GetScopedName(), this->pid);
            }

            // Create the node
            this->node = transport::NodePtr(new transport::Node());

            this->node->Init(this->model->GetWorld()->Name());

            // Create a topic name
            std::string topicName = "~/" + this->model->GetName() + "/angle_cmd";
            // Subscribe to the topic, and register a callback
            this->sub = this->node->Subscribe(topicName, &JointSubscriberPlugin::OnMsg, this);
        }

        /// \brief Set the angle of joint
        /// \param[in] rad is value of an angle in Radians
        void SetJointAngle(std::string joint, const double &rad) {
            // Set the joint's target velocity.
            jointController->SetPositionTarget(joint, rad);
        }
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(JointSubscriberPlugin)
}
#endif
