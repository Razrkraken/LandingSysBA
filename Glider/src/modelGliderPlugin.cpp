//
// Created by Alejandro Aguilar on 24.09.20.
//

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo {
    /**
     * A plugin to control the SUAV via gazebo topics.
     */
    class GliderPlugin : public ModelPlugin {
    public:
        GliderPlugin() {}

        /***
         * The load function is called by Gazebo when the plugin is inserted into simulation
         *
         * @param model A pointer to the model that this plugin is attached to.
         * @param sdf A pointer to the plugin's SDF element.
         */
        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
            // Store the model pointer for convenience.
            gzmsg << "Loading Glider Plugin\n" ;

            this->model = model;

            std::array<std::string, 4> JOINT_NAMES = {
                    "elevator_joint",
                    "rudder_joint",
                    "left_aileron_joint",
                    "right_aileron_joint",
            };

            for (auto name : JOINT_NAMES) {
                physics::JointPtr joint = model->GetJoint("SUAV_glider::SUAV_glider::" + name);
                if (joint == nullptr) {
                    gzerr << "Missing " << name << " joint\n";
                    return;
                }
                 gzmsg << "Found " << name << " joint\n";
                joints[name] = joint;
                this->model->GetJointController()->SetPositionPID(
                        joint->GetScopedName(), common::PID(0.035, 0.001, 0.001, 0, 0, 4, -4));
                this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), 0);

            }

            // Create the node
            this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
            this->node->Init(this->model->GetWorld()->GetName());
#else
            this->node->Init(this->model->GetWorld()->Name());
#endif

            // Create the topic names
            // Topics to Subscribe

            gzmsg << "Model name: " << this->model->GetName() << "\n ";
            std::string elevatorTopicName = "~/" + this->model->GetName() + "/elevator_angle";
            std::string leftAileronTopicName = "~/" + this->model->GetName() + "/left_aileron_angle";
            std::string rightAileronTopicName = "~/" + this->model->GetName() + "/right_aileron_angle";
            std::string rudderTopicName = "~/" + this->model->GetName() + "/rudder_angle";
                // Topics to Publish
            std::string posesTopicName = "~/" + this->model->GetName() + "/my_pose";
            std::string velTopicName = "~/" + this->model->GetName() + "/my_vel";
            std::string elevTopicName = "~/" + this->model->GetName() + "/my_elev";

                // Subscribe to the topic and register a callback
            this->elevatorSubscriber = this->node->Subscribe(
                    elevatorTopicName, &GliderPlugin::OnMsgElevator, this);
            this->rudderSubscriber = this->node->Subscribe(
                    rudderTopicName, &GliderPlugin::OnMsgRudder, this);
            this->leftAileronSubscriber = this->node->Subscribe(
                    leftAileronTopicName, &GliderPlugin::OnMsgLeftAileron, this);
            this->rightAileronSubscriber = this->node->Subscribe(
                    rightAileronTopicName, &GliderPlugin::OnMsgRightAileron, this);

            // create Publisher
            this->positionPublisher = this->node->Advertise<msgs::Vector3d>(posesTopicName,1000,5);
            this->velPublisher = this->node->Advertise<msgs::Vector3d>(velTopicName,1000,5);
            this->elevPublisher = this->node->Advertise<msgs::Vector3d>(elevTopicName,1000,5);

//            gazebo::transport::Publisher::PublishImpl
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&GliderPlugin::posHandler, this));

            this->update2Connection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&GliderPlugin::velHandler, this));

            this->update3Connection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&GliderPlugin::elevPosHandler, this));

//             Set the initial velocity of the cessna
            if (sdf->HasElement("initial_vel")){
                math::Vector3 initial_vel = sdf->GetElement("initial_vel")->Get<math::Vector3>();
                model->GetLink("SUAV_glider::fuselage")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::left_wing")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::right_wing")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::left_aileron")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::right_aileron")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::elevator")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::rudder")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::battery")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::vertical_stab")->SetLinearVel(initial_vel);
                model->GetLink("SUAV_glider::horizontal_stab")->SetLinearVel(initial_vel);
            }
        }

        void posHandler()
        {
            const math::Vector3 pose = this->model->GetLink("SUAV_glider::fuselage")->GetWorldCoGPose().pos;
            msgs::Vector3d pose_msgs;
            pose_msgs.set_x(pose.x);
            pose_msgs.set_y(pose.y);
            pose_msgs.set_z(pose.z);
            positionPublisher->Publish(pose_msgs);
        }

        void velHandler(){
            math::Vector3 vel = this->model->GetLink("SUAV_glider::fuselage")->GetWorldCoGLinearVel();
            msgs::Vector3d vel_msgs;
            vel_msgs.set_x(vel.x);
            vel_msgs.set_y(vel.y);
            vel_msgs.set_z(vel.z);
            velPublisher->Publish(vel_msgs);
        }

        void elevPosHandler(){
            math::Angle vel = this->model->GetJoint("SUAV_glider::SUAV_glider::elevator_joint")->GetAngle(0);
            msgs::Vector3d vel_msgs;
            vel_msgs.set_x(vel.Radian());
            vel_msgs.set_y(0.0);
            vel_msgs.set_z(0.0);
            elevPublisher->Publish(vel_msgs);
        }

    private:

        /**
         * Set the target value of the elevator PID controller.
         *
         * @param msg A vector containing the target value as the x component.
         */
        void OnMsgElevator(ConstVector2dPtr &msg) {
            this->model->GetJointController()->SetPositionTarget(joints["elevator_joint"]->GetScopedName(), msg->x());
        }

        /**
         * Set the target value of the elevator PID controller.
         *
         * @param msg A vector containing the target value as the x component.
         */
        void OnMsgLeftAileron(ConstVector2dPtr &msg) {
            this->model->GetJointController()->SetPositionTarget(joints["left_aileron_joint"]->GetScopedName(), msg->x());
        }

        /**
         * Set the target value of the elevator PID controller.
         *
         * @param msg A vector containing the target value as the x component.
         */
        void OnMsgRightAileron(ConstVector2dPtr &msg) {
            this->model->GetJointController()->SetPositionTarget(joints["right_aileron_joint"]->GetScopedName(), msg->x());
        }

        /**
         * Set the target value of the rudder PID controller.
         *
         * @param msg A vector containing the target value as the x component.
         */
        void OnMsgRudder(ConstVector2dPtr &_msg) {
            this->model->GetJointController()->SetPositionTarget(joints["rudder_joint"]->GetScopedName(), _msg->x());
        }

        /**
         * Pointer to the model.
         */
        physics::ModelPtr model;

        /**
         * A node used for transport
         */
        transport::NodePtr node;

        /**
         * A subscriber to the Gazebo elevator topic.
         */
        transport::SubscriberPtr elevatorSubscriber;

        /**
         * A subscriber to the Gazebo rudder topic.
         */
        transport::SubscriberPtr rudderSubscriber;

        /**
        * A subscriber to the Gazebo rudder topic.
        */
        transport::SubscriberPtr leftAileronSubscriber;

        /**
        * A subscriber to the Gazebo rudder topic.
        */
        transport::SubscriberPtr rightAileronSubscriber;

        transport::PublisherPtr positionPublisher;

        transport::PublisherPtr velPublisher;

        transport::PublisherPtr elevPublisher;

        std::map<std::string, physics::JointPtr> joints;

        event::ConnectionPtr updateConnection;

        event::ConnectionPtr update2Connection;

        event::ConnectionPtr update3Connection;

    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(GliderPlugin)

}
