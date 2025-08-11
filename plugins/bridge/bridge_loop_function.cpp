#include "bridge_loop_function.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;

void CBridgeLoopFunction::Init(TConfigurationNode &)
{
    LOG << "[LoopFunction] Init() called." << std::endl;

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    RosNode_ = std::make_shared<rclcpp::Node>("bridge_loop_function");

    // 创建 Executor 并运行
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(RosNode_);
    std::thread([executor]()
                { executor->spin(); })
        .detach();

    // 为每个机器人注册 trigger_signal 订阅
    for (const auto &[id, _] : GetSpace().GetEntitiesByType("foot-bot"))
    {
        std::string topic = "/" + id + "/trigger_signal";

        auto sub = RosNode_->create_subscription<std_msgs::msg::Header>(
            topic, 10,
            [this, id](const std_msgs::msg::Header::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(Mutex_);
                TriggerTimestamps_[id] = msg->stamp;
                CV_.notify_one();
            });

        TriggerSubscribers_[id] = sub;
        TriggerTimestamps_[id] = rclcpp::Time(0); // 初始化时间戳为 0

        LOG << "[LoopFunction] Subscribed to: " << topic << std::endl;
    }

    TriggerPublisher_ = RosNode_->create_publisher<std_msgs::msg::Header>("/trigger_controller", 10);
    clockPublisher_ = RosNode_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
}

void CBridgeLoopFunction::PreStep()
{
    std::unique_lock<std::mutex> lock(Mutex_);

    const double time_per_step_sec = 0.01;
    uint64_t step = GetSpace().GetSimulationClock();
    rclcpp::Time expected_stamp(static_cast<int64_t>(step * time_per_step_sec * 1e9));
    const double expected_secs = expected_stamp.seconds();

    constexpr auto wait_timeout = std::chrono::milliseconds(10);
    const int max_attempts = 20;

    bool condition_met = false;

    std::vector<std::string> missing_robots;

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        condition_met = CV_.wait_for(lock, wait_timeout, [&]()
                                     {
        missing_robots.clear();

        for (const auto &pair : TriggerTimestamps_)
        {
            double trigger_secs = pair.second.seconds();
            if (std::abs(trigger_secs - expected_secs) - time_per_step_sec > 1e-6)
            {
                missing_robots.push_back(pair.first);
            }
        }

        return missing_robots.empty(); });

        if (condition_met)
            break;

        std::cout << "[LoopFunction] Re-sending trigger at attempt "
                  << (attempt + 1) << " for time " << expected_secs << std::endl;

        if (!missing_robots.empty())
        {
            std::cout << "[LoopFunction] Missing triggers from robots: ";
            for (const auto &id : missing_robots)
            {
                std::cout << id << " ";
            }
            std::cout << std::endl;
        }

        std_msgs::msg::Header msg;
        msg.stamp = expected_stamp;
        TriggerPublisher_->publish(msg);
    }

    if (!condition_met)
    {
        std::cerr << "[LoopFunction] Timeout waiting for triggers at time "
                  << expected_secs << ", skipping." << std::endl;
    }

    const auto &sim = argos::CSimulator::GetInstance();
    argos::CPhysicsEngine &engine = sim.GetPhysicsEngine("dyn2d");
    argos::Real sim_time = sim.GetSpace().GetSimulationClock() * engine.GetPhysicsClockTick();
    rclcpp::Time ros_sim_time(static_cast<uint64_t>(sim_time * 1e9));

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = ros_sim_time;

    if (clockPublisher_)
    {
        clockPublisher_->publish(clock_msg);
    }
}

void CBridgeLoopFunction::PostStep()
{
    uint64_t step = GetSpace().GetSimulationClock();

    double sim_time_sec = step * 0.01;

    std_msgs::msg::Header msg;
    msg.stamp = rclcpp::Time(static_cast<int64_t>(sim_time_sec * 1e9));

    if (TriggerPublisher_->get_subscription_count() == 0)
    {
        RCLCPP_WARN(RosNode_->get_logger(),
                    "[PostStep] No subscribers to /trigger_controller at time %.3f, skipping publish.",
                    sim_time_sec);
        return;
    }

    try
    {
        TriggerPublisher_->publish(msg);

        RCLCPP_DEBUG(RosNode_->get_logger(),
                     "[PostStep] Published /trigger_controller at %.3f (step %lu)",
                     sim_time_sec, step);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(RosNode_->get_logger(),
                     "[PostStep] Failed to publish /trigger_controller: %s",
                     e.what());
    }
}

REGISTER_LOOP_FUNCTIONS(CBridgeLoopFunction, "bridge_loop_function")
