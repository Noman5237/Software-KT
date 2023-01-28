#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "temperature_control_systems_interfaces/srv/get_current_temperature.hpp"

using namespace std;

class TemperatureSystemsControllerNode : public rclcpp::Node {
private:
    // context prefix
    const string context_prefix = "temperature_control_systems";
    // store the current temperature
    short current_temperature;
    rclcpp::TimerBase::SharedPtr temperature_monitor_timer;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr temperature_publisher;

public:
    TemperatureSystemsControllerNode() : Node("temperature_control_systems") {
        // initialize the current temperature with random value between 15 and 100
        current_temperature = rand() % 85 + 15;
        // create a publisher
        temperature_publisher = this->create_publisher<std_msgs::msg::Int16>(context_prefix + "__temperature", 10);
        // create a monitor timer
        temperature_monitor_timer = this->create_wall_timer(1s, [this] { temperature_monitor_callback(); });
    }

private:
    void temperature_monitor_callback() {
        // publish the current temperature
        auto message = std_msgs::msg::Int16();
        message.data = current_temperature;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", to_string(message.data).c_str());
        temperature_publisher->publish(message);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureSystemsControllerNode>());
    rclcpp::shutdown();
    return 0;
}