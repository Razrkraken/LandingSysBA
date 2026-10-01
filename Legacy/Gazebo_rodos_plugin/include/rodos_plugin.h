#pragma once
#include <gazebo/gazebo.hh>

/**
 * An interface implemented by all GazeboTopics to be initialized with the Gazebo node.
 */
class GzTopicInitializer {
public:
    GzTopicInitializer();

    /**
     * Initialize the GzTopicInitializer with the Gazebo node.
     *
     * @param node The Gazebo node.
     */
    virtual void init(gazebo::transport::NodePtr node) = 0;

    /**
     * @return A readonly reference to the list of all GzTopicInitializer instances.
     */
    static const std::list<GzTopicInitializer *> &getGzTopicList();
private:
    /**
     * The list of all GzTopicInitializer instances.
     */
    static std::list<GzTopicInitializer *> gzTopicList;
};

namespace gazebo {

    /**
     * A plugin to connect RODOS and Gazebo topics.
     */
    class RodosPlugin : public WorldPlugin {
    public:
        RodosPlugin();

        /**
         * The load function is called by Gazebo when the world is loaded.
         *
         * @param world A pointer to the simulated world.
         * @param sdf A pointer to the plugin's SDF element.
         */
        void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override;

    private:
        /**
         * Handler for signals that forwards the signals to the RODOS main thread.
         *
         * @param signal The signal that happened.
         */
        static void signalHandler(int signal);

        /**
         * Start RODOS.
         */
        static void rodosSystemMain();

        /**
         * Initialize the GazeboTopics with the Gazebo node.
         */
        void initConnectors();

        /**
         * The Gazebo node used for transport.
         */
        transport::NodePtr node;

        /**
         * The RODOS main thread.
         */
        static pthread_t rodosThreadId;
    };
}
