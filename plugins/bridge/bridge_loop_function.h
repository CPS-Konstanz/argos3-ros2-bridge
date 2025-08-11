#ifndef BRIDGE_LOOP_FUNCTION_H
#define BRIDGE_LOOP_FUNCTION_H

#include <argos3/core/simulator/loop_functions.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "rosgraph_msgs/msg/clock.hpp"

#include <unordered_map>
#include <mutex>

class CBridgeLoopFunction : public argos::CLoopFunctions
{
public:
    void Init(argos::TConfigurationNode &t_tree) override;
    void PreStep() override;
    void PostStep() override;

private:
    std::shared_ptr<rclcpp::Node> RosNode_;
    // sub
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr> TriggerSubscribers_;
    // pub
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr TriggerPublisher_;
    // sim clock publisher
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPublisher_;

    std::unordered_map<std::string, bool> TriggerFlags_;
    std::unordered_map<std::string, rclcpp::Time> TriggerTimestamps_;

    std::mutex Mutex_;
    std::condition_variable CV_;
};

#endif // BRIDGE_LOOP_FUNCTION_H
