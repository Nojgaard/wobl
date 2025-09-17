#include <wobl/node.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <csignal>

using namespace wobl;

// Global atomic flag for signal interruption
static std::atomic<bool> signal_interrupted{false};

// Signal handler that sets the interruption flag
static void signal_handler(int signal)
{
    signal_interrupted.store(true);
}

Node::Node()
    : session_(zenoh::Session::open(zenoh::Config::create_default(), zenoh::Session::SessionOptions::create_default())), 
    is_open_(true)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
}

Node::~Node()
{
    close();
}

bool Node::is_open() const
{
    return is_open_.load() && !signal_interrupted.load();
}

void Node::spin()
{
    while (is_open())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void Node::close()
{
    if (!is_open_.exchange(false))
    {
        return; // Already closed
    }
    is_open_.store(false);

    // Stop all timer threads
    for (auto &thread : timer_threads_)
    {
        if (thread && thread->joinable())
        {
            thread->join();
        }
    }
    timer_threads_.clear();
    session_.close();
}

int Node::add_pub(const std::string &key)
{
    zenoh::ZResult result;
    auto pub = session_.declare_publisher(zenoh::KeyExpr(key), zenoh::Session::PublisherOptions::create_default(), &result);
    if (result != Z_OK)
    {
        throw std::runtime_error("Failed to create publisher for key: " + key);
    }

    publishers_.push_back(std::move(pub));
    return static_cast<int>(publishers_.size() - 1);
}

int Node::add_sub(const std::string &key,
                  std::function<void(const zenoh::Sample &)> callback)
{
    zenoh::ZResult result;
    auto sub = session_.declare_subscriber(zenoh::KeyExpr(key), std::move(callback), []() { /* on_drop */ }, zenoh::Session::SubscriberOptions::create_default(), &result);
    if (result != Z_OK)
        throw std::runtime_error("Failed to create subscriber for key: " + key);

    subscribers_.push_back(std::move(sub));
    return static_cast<int>(subscribers_.size() - 1);
}

int Node::add_sub(const std::string &key,
                  google::protobuf::Message *message)
{
    auto callback = [message](const zenoh::Sample &sample)
    {
        auto &payload = sample.get_payload();

        if (payload.size() == 0)
            return;

        auto data = payload.as_vector();
        message->ParseFromArray(data.data(), static_cast<int>(data.size()));
    };

    zenoh::ZResult result;
    auto sub = session_.declare_subscriber(zenoh::KeyExpr(key), std::move(callback), []() { /* on_drop */ }, zenoh::Session::SubscriberOptions::create_default(), &result);
    if (result != Z_OK)
        throw std::runtime_error("Failed to create subscriber for key: " + key);

    subscribers_.push_back(std::move(sub));
    return static_cast<int>(subscribers_.size() - 1);
}

void Node::send(int pub_id, const google::protobuf::Message &message)
{
    if (pub_id < 0 || pub_id >= static_cast<int>(publishers_.size()))
        throw std::invalid_argument("Invalid publisher ID");

    std::string serialized = message.SerializeAsString();
    zenoh::Bytes payload(std::move(serialized));

    publishers_[pub_id].put(std::move(payload));
}

void Node::add_timer(std::function<void()> callback, double frequency_hz)
{
    auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / frequency_hz));

    auto timer_func = [this, callback = std::move(callback), period]()
    {
        auto next_call = std::chrono::steady_clock::now() + period;

        while (is_open())
        {
            callback();

            std::this_thread::sleep_until(next_call);
            next_call += period;
        }
    };

    auto thread = std::make_unique<std::thread>(timer_func);
    timer_threads_.push_back(std::move(thread));
}

double Node::clock() const
{
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}