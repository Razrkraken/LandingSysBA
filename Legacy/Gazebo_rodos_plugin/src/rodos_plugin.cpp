#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include <dlfcn.h>
#include <GazeboTopic.h>
#include "rodos_plugin.h"

/**
 * The type of the rodos main function.
 */
typedef int (*rodosMainFunc)(int, char **);

namespace gazebo {
    pthread_t RodosPlugin::rodosThreadId = 0;

    RodosPlugin::RodosPlugin() {
        signal(SIGALRM, RodosPlugin::signalHandler);
        signal(SIGUSR1, RodosPlugin::signalHandler);
        signal(SIGIO, RodosPlugin::signalHandler);
        if (rodosThreadId == 0) {
            std::thread rodosMainThread(&RodosPlugin::rodosSystemMain);
            rodosThreadId = rodosMainThread.native_handle();
            rodosMainThread.detach();
        }
    }

    void RodosPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
        this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(world->GetName());
#else
        this->node->Init(world->Name());
#endif
        initConnectors();
    }

    void RodosPlugin::signalHandler(int signal) {
        if (rodosThreadId != 0) {
            pthread_kill(RodosPlugin::rodosThreadId, signal);
        }
    }

    void RodosPlugin::rodosSystemMain() {
        pthread_setname_np(pthread_self(), "RODOS_main_thread");
        auto rodosMain = (rodosMainFunc) dlsym(RTLD_DEFAULT, "main");
        if (rodosMain == nullptr) {
            gzerr << "Unable to find RODOS main function!" << std::endl;
        } else {
            rodosMain(0, nullptr);
        }
    }

    void RodosPlugin::initConnectors() {
        for (auto gzTopic: GzTopicInitializer::getGzTopicList()) {
            gzTopic->init(this->node);
        }
    }

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_WORLD_PLUGIN(RodosPlugin)
}

GzTopicInitializer::GzTopicInitializer() {
    gzTopicList.push_back(this);
}

const std::list<GzTopicInitializer *> &GzTopicInitializer::getGzTopicList() {
    return gzTopicList;
}

std::list<GzTopicInitializer *> GzTopicInitializer::gzTopicList;
