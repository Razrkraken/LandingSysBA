#include <GazeboTopic.h>
#include <gazebo/msgs/imu.pb.h>
#include <rodos.h>

static Application exampleApplication("ExampleApplication");

// The topics that will be shared between Gazebo and RODOS.
GazeboTopic<gazebo::msgs::IMU> imuTopic(10, "~/cessna_c172/cessna_c172/body/IMU_Fuselage/imu");
GazeboTopic<gazebo::msgs::Vector2d> rudderCmdTopic(11, "~/cessna_c172/rudder_angle");


/**
 * A subscriber that demonstrates how to read values from a Gazebo topic.
 */
class IMUSub : public Subscriber {
public:
    IMUSub() : Subscriber(imuTopic, "ImuSubscriber") {}

    uint32_t put(const uint32_t topicId, const size_t len, void *data, const NetMsgInfo &netMsgInfo) override {
        gazebo::msgs::IMU *gazeboData = (gazebo::msgs::IMU *) data;
        gazebo::msgs::Vector3d acceleration = gazeboData->linear_acceleration();
//        PRINTF("X:%f Y:%f Z:%f\n", acceleration.x(), acceleration.y(), acceleration.z());
        return 1;
    }
} imuSubscriber;

/**
 * An example thread that controls the rudder (from left to right).
 */
class RudderControlThread : public StaticThread<> {
public:
    RudderControlThread() : StaticThread("RudderControlThread") {}

    [[noreturn]] void run() override {
        gazebo::msgs::Vector2d cmd;
        while (true) {
            if (cmd.x() >= 0.5) {
                cmd.set_x(-0.5);
            } else {
                cmd.set_x(0.5);
            }
            cmd.set_y(0);
            rudderCmdTopic.publish(cmd);
            suspendCallerUntil(NOW() + SECONDS);
        }
    }
} rudderControlThread;
