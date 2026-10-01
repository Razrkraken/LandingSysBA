#pragma once

#ifdef TIME_SLICE_FOR_SAME_PRIORITY
#  error "RODOS must be included after gazebo"
#endif

#include <gazebo/gazebo.hh>
#include <rodos.h>
#include <queue>
#include <condition_variable>
#include <utility>
#include "rodos_plugin.h"

template<class T>
class GazeboTopic;

template<class T>
class GazeboTopicThread : public StaticThread<> {
public:
    /**
     * A thread that forwards messages from a Gazebo Topic to a RODOS topic.
     *
     * @param topic The GazeboTopic instance top forward to.
     */
    GazeboTopicThread(GazeboTopic<T> *topic) : StaticThread(
            (topic->getName() + std::string("_publisher")).c_str(),
            DEFAULT_THREAD_PRIORITY), topic(topic) {}

    void run() override {
        std::unique_lock<std::mutex> lock(topic->queueLock);
        while (running) {
            topic->queueWait.wait(lock);
            while (!topic->toRodosQueue.empty()) {
                T data = topic->toRodosQueue.front();
                currentMsg = &data;
                topic->publish(data);
                topic->toRodosQueue.pop();
                currentMsg = nullptr;
            }
        }
    }

    /**
     * Stop this Gazebo topic thread.
     */
    void stop() {
        running = false;
        topic->queueWait.notify_all();
    }

    /**
     * Check if a message is the last message send from Gazebo.
     *
     * @param msg The message to compare.
     * @return true, if the message is the last message send from Gazebo, false otherwise.
     */
    bool isMsgFromGazebo(const T *msg) const {
        return msg == currentMsg;
    }

private:
    /**
     * The last message send from Gazebo.
     */
    T *currentMsg = nullptr;

    /**
     * The RODOS topic to forward the messages to.
     */
    GazeboTopic<T> *topic;

    /**
     * Whether the thread should be running or not.
     */
    bool running = true;
};


/**
 * A putter to a RODOS GazeboTopic that forwards messages from RODOS to Gazebo.
 *
 * @tparam T The type of the messages.
 */
template<class T>
class GazeboTopicPutter : public Putter {
public:
    explicit GazeboTopicPutter(GazeboTopic<T> *gazeboTopic) : topic(gazeboTopic) {}

    bool putGeneric(uint32_t topicId, size_t len, const void *msg,
            const NetMsgInfo &netMsgInfo) override {
        auto data = (const T *) msg;
        // Filter out messages that were originally send from Gazebo
        // to prevent an infinite message loop.
        if (publisher == nullptr || topic->publisherThread.isMsgFromGazebo(data)) { return false; }
        std::unique_lock<std::mutex> lock(topic->eventsSentToGazeboLock);
        topic->eventsSentToGazebo.push_back(*data);
        publisher->Publish(*data, true);
        return true;
    }

    /**
     * Initialize the putter and link it to the Gazebo publisher.
     *
     * @param gazeboPublisher The Gazebo publisher to forward messages to.
     */
    void init(gazebo::transport::PublisherPtr gazeboPublisher) {
        publisher = std::move(gazeboPublisher);
    }

private:
    /**
     * The GazeboTopic forward messages from.
     */
    GazeboTopic<T> *topic;

    /**
     * The publisher that publishes to the Gazebo topic.
     */
    gazebo::transport::PublisherPtr publisher = nullptr;
};

/**
 * A RODOS topic connected to a Gazebo topic.
 *
 * The topic will automatically create a Gazebo topic with the same name, if it doesn't exist yet.
 * Messages send to this topic from RODOS will be forwarded to the Gazebo topic
 * and messages that arrive on the Gazebo topic will be forwarded to this RODOS topic.
 *
 * @tparam T The type of the messages that will be send and received on this topic.
 */
template<class T>
class GazeboTopic : public Topic<T>, GzTopicInitializer {
    friend class GazeboTopicThread<T>;
    friend class GazeboTopicPutter<T>;

public:
    /**
     * Create a new GazeboTopic.
     *
     * @param id The id of the topic.
     * @param name The name of the Rodos and Gazebo topic.
     * @param onlyLocal See TopicInterface::onlyLocal.
     */
    GazeboTopic(int64_t id, const char *name, bool onlyLocal = false) :
            Topic<T>(id, name, onlyLocal), publisherThread(this), gazeboPutter(this),
            rodosSub(*this, gazeboPutter, (this->getName() + std::string("_subscriber")).c_str()) {
    }

    ~GazeboTopic() override {
        publisherThread.stop();
    }

    /**
     * Initialize the GazeboTopic with the Gazebo node.
     *
     * @param node The Gazebo node.
     */
    void init(gazebo::transport::NodePtr node) override {
        gzmsg << "Loading GazeboTopic: " << this->getName() << std::endl;
        this->gazeboSub = node->Subscribe(this->getName(), &GazeboTopic<T>::onGazeboMsg, this);
        this->gazeboPutter.init(node->Advertise<T>(this->getName(), 50, 50));
    }

private:
    /**
     * Handle messages from the Gazebo thread.
     *
     * @param msg The message date.
     */
    void onGazeboMsg(const boost::shared_ptr<T const> &msg) {
        std::unique_lock<std::mutex> lock(eventsSentToGazeboLock);
        // Filter out messages that were originally send from RODOS
        // to prevent an infinite message loop.
        const auto &dataPosition = std::find_if(
                eventsSentToGazebo.begin(), eventsSentToGazebo.end(), [&](T item) {
            return std::memcmp(msg.get(), &item, sizeof(T)) == 0;
        });
        if (dataPosition != eventsSentToGazebo.end()) {
            eventsSentToGazebo.erase(dataPosition);
            return;
        }
        // RODOS requires messages to be inserted into topics from a thread
        // that was started by RODOS. We defer the inserting to the GazeboTopicThread.
        std::unique_lock<std::mutex> uniqueLock(queueLock);
        toRodosQueue.emplace(*msg);
        queueWait.notify_one();
    }

    /**
     * The putter forwarding messages from RODOS to Gazebo.
     */
    GazeboTopicPutter<T> gazeboPutter;

    /**
     * The subscriber forwarding RODOS messages to the GazeboTopicPutter.
     */
    Subscriber rodosSub;

    /**
     * The subscriber forwarding Gazebo messages to the this GazeboTopic.
     */
    gazebo::transport::SubscriberPtr gazeboSub;

    /**
     * The thread that publishes messages from Gazebo to the RODOS topic.
     */
    GazeboTopicThread<T> publisherThread;

    /**
     * A queue of messages from Gazebo which must be forwarded to the RODOS topic
     */
    std::queue<T> toRodosQueue;

    /**
     * A list of messages that were send to the Gazebo topic.
     */
    std::list<T> eventsSentToGazebo;

    /**
     * A lock for accessing the GazeboTopic::toRodosQueue.
     */
    std::mutex queueLock;

    /**
     * A lock for accessing the GazeboTopic::eventsSentToGazebo list.
     */
    std::mutex eventsSentToGazeboLock;

    /**
     * A condition that allows to wait for a new element in the GazeboTopic::toRodosQueue.
     */
    std::condition_variable queueWait;
};
