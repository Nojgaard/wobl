#pragma once

#include <zenoh.hxx>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <atomic>
#include <google/protobuf/message.h>

namespace wobl {
    class Node {
    public:
        Node();
        ~Node();
        
        // Core functionality
        void spin();
        void close();
        
        // Publisher/Subscriber management
        int add_pub(const std::string& key);
        int add_sub(const std::string& key, 
                    std::function<void(const zenoh::Sample&)> callback);
        int add_sub(const std::string& key, 
                    google::protobuf::Message* message);
        
        // Send message
        void send(int pub_id, const google::protobuf::Message& message);
        
        // Timer functionality
        void add_timer(std::function<void()> callback, double frequency_hz);
        double clock() const;
        bool is_open() const;
        
    private:
        zenoh::Session session_;
        std::vector<zenoh::Publisher> publishers_;
        std::vector<zenoh::Subscriber<void>> subscribers_;
        std::vector<std::unique_ptr<std::thread>> timer_threads_;
        std::atomic<bool> is_open_;
    };
}