#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp" // Required for dynamic subscription
#include <map>
#include <vector>
#include <string>
#include <chrono>
#include <atomic>
#include "minion_diagnostic_msg/msg/system_monitor_data.hpp"
#include "minion_diagnostic_msg/msg/node_status.hpp"        
#include "minion_diagnostic_msg/msg/topic_status.hpp"

using namespace std::chrono_literals;

class SystemMonitor : public rclcpp::Node
{
public:
    SystemMonitor() : Node("system_monitor_node")
    {
        status_publisher_ = this->create_publisher<minion_diagnostic_msg::msg::SystemMonitorData>("system_status", 10);
        // Set up a periodic timer to run the system check
        timer_ = rclcpp::create_timer(this, this->get_clock(), 3s, std::bind(&SystemMonitor::monitor_system, this));
        RCLCPP_INFO(this->get_logger(), "System Monitor Node Initialized. Running check every 5s.");
    }

private:
    rclcpp::Publisher<minion_diagnostic_msg::msg::SystemMonitorData>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void monitor_system();
    bool check_active_publishing_generic(const std::string& topic_name, const std::string& topic_type_str);
    std::string normalize_node_name(const std::string& name) const;
};

void SystemMonitor::monitor_system() {
    minion_diagnostic_msg::msg::SystemMonitorData status_msg ;
    status_msg.stamp = this->get_clock()->now();

    // Get all running nodes
    std::vector<std::string> node_names = this->get_node_names();

    // Get all topics in the graph
    auto topic_types = this->get_topic_names_and_types();

    // Map to store results: Node -> Topic -> Publishing Status
    std::map<std::string, std::map<std::string, bool>> node_topic_status;

    for (const auto& node_name : node_names)
    {
        // Skip the internal ROS 2 daemon
        if (node_name.find("/_ros2cli_daemon_") == 0) {
            continue;
        }

        minion_diagnostic_msg::msg::NodeStatus current_node_status;
        current_node_status.node_name = node_name;

        const std::string normalized_node_name = normalize_node_name(node_name);
        // Iterate over all topics in the graph
        for (const auto& topic_pair : topic_types)
        {
            const std::string& topic_name = topic_pair.first;
            const std::string& topic_type_str = topic_pair.second.front();
            
            // Get detailed info (publishers, node GID, etc.) for this topic
            auto topic_infos = this->get_publishers_info_by_topic(topic_name);

            // Check if the current node is listed as a publisher for this topic
            for (const auto& info : topic_infos)
            {
                const std::string info_normalized_name = normalize_node_name(info.node_name());
                if (info_normalized_name == normalized_node_name)
                {
                    // Found a matching publisher! Now verify if it's active.
                    // We use the generic subscription check here:
                    bool is_active = check_active_publishing_generic(topic_name, topic_type_str);

                    minion_diagnostic_msg::msg::TopicStatus current_topic_status;
                    current_topic_status.topic_name = topic_name;
                    current_topic_status.topic_type = topic_type_str;
                    current_topic_status.is_publishing = is_active;
                    
                    // Add topic status to the node status array
                    current_node_status.published_topics.push_back(current_topic_status);
                    
                    break;
                }
            }
        }

        status_msg.node_statuses.push_back(current_node_status);
    }

    status_publisher_->publish(status_msg);
}

bool SystemMonitor::check_active_publishing_generic(const std::string& topic_name, const std::string& topic_type_str) {
    // Use an atomic flag for thread-safe signaling from the callback
    std::atomic<bool> message_received{false};
    
    // Create a GenericSubscription (subscribes to any type)
    // We use a lambda to set the flag when a message (serialized) is received.
    auto temp_check_node = std::make_shared<rclcpp::Node>("temp_topic_checker"); // temp node

    auto sub = temp_check_node->create_generic_subscription(
        topic_name,
        topic_type_str,
        rclcpp::QoS(1), // Minimal QoS depth
        [&](std::shared_ptr<rclcpp::SerializedMessage>) {
            // This callback proves a message was physically sent and received
            message_received = true; 
        }
    );

    rclcpp::executors::SingleThreadedExecutor local_executor;
    local_executor.add_node(temp_check_node);
    rclcpp::Clock clock;
    rclcpp::Time start_time = clock.now();
    rclcpp::Duration timeout = 100ms; // Check for half a second (adjust as needed)

    // Timed loop using non-blocking spin (rclcpp::spin_some)
    while (rclcpp::ok() && (clock.now() - start_time) < timeout)
    {
        if (message_received) {
            break;
        }
        // Processes any pending callbacks (including the message_received callback)
        local_executor.spin_some(10ms);
    }
    
    // cleaning up the subscription.
    local_executor.remove_node(temp_check_node);

    return message_received;
}

std::string SystemMonitor::normalize_node_name(const std::string& name) const {
    if (name.empty()) {
        return name;
    }
    if (name.front() == '/') {
        return name.substr(1);
    }
    return name;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Use rclcpp::spin() to keep the monitor node running and trigger the timer
    rclcpp::spin(std::make_shared<SystemMonitor>());
    rclcpp::shutdown();
    return 0;
}