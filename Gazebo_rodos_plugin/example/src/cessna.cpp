#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


namespace gazebo {
    /**
     * A plugin to control the Cessna via gazebo topics.
     */
    class CessnaPlugin : public ModelPlugin {
    public:
        CessnaPlugin() {}

        /***
         * The load function is called by Gazebo when the plugin is inserted into simulation
         *
         * @param model A pointer to the model that this plugin is attached to.
         * @param sdf A pointer to the plugin's SDF element.
         */
        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
            // Store the model pointer for convenience.
            this->model = model;

            std::array<std::string, 6> JOINT_NAMES = {
                    "elevators_joint",
                    "rudder_joint",
                    "left_flap_joint",
                    "left_aileron_joint",
                    "right_flap_joint",
                    "right_aileron_joint",
            };
            for (auto name : JOINT_NAMES) {
                physics::JointPtr joint = model->GetJoint("cessna_c172::" + name);
                if (joint == nullptr) {
                    gzerr << "Missing " << name << " joint\n";
                    return;
                }
                joints[name] = joint;
                this->model->GetJointController()->SetPositionPID(
                        joint->GetScopedName(), common::PID(50, 0.1, 1, 0, 0, 20000, -20000));
                this->model->GetJointController()->SetPositionTarget(
                        joint->GetScopedName(), 0);
            }

            // Create the node
            this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
            this->node->Init(this->model->GetWorld()->GetName());
#else
            this->node->Init(this->model->GetWorld()->Name());
#endif

            // Create the topic names
            std::string elevatorTopicName = "~/" + this->model->GetName() + "/elevator_angle";
            std::string rudderTopicName = "~/" + this->model->GetName() + "/rudder_angle";


            // Subscribe to the topic and register a callback
            this->elevatorSubscriber = this->node->Subscribe(
                    elevatorTopicName, &CessnaPlugin::OnMsgElevator, this);
            this->rudderSubscriber = this->node->Subscribe(
                    rudderTopicName, &CessnaPlugin::OnMsgRudder, this);


            // Set the initial velocity of the cessna
            if (sdf->HasElement("initial_vel")) {
                math::Vector3 initial_vel = sdf->GetElement("initial_vel")->Get<math::Vector3>();
                model->GetLink("cessna_c172::body")->SetLinearVel(initial_vel);
            }

        }

    private:
        /**
         * Set the target value of the elevator PID controller.
         * 
         * @param msg A vector containing the target value as the x component.
         */
        void OnMsgElevator(ConstVector2dPtr &msg) {
            this->model->GetJointController()->SetPositionTarget(joints["elevators_joint"]->GetScopedName(), msg->x());
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
         * The joints of the cessna by name.
         */
        std::map<std::string, physics::JointPtr> joints;
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(CessnaPlugin)
}
