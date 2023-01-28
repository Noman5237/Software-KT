#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int16.hpp"

#include "temperature_control_systems_interfaces/srv/get_current_temperature.hpp"
#include "temperature_control_systems_interfaces/srv/increment_decrement_temperature.hpp"
#include "temperature_control_systems_interfaces/action/set_temperature.hpp"

using namespace std;
using namespace temperature_control_systems_interfaces::srv;
using namespace temperature_control_systems_interfaces::action;

class TemperatureSystemsControllerNode : public rclcpp::Node {
private:
	// context prefix
	const string contextPrefix = "temperature_control_systems";
	// current temperature
	short currentTemperature;
	// timer to monitor the temperature
	rclcpp::TimerBase::SharedPtr temperatureMonitorTimer;
	// publisher to publish the temperature
	rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr temperaturePublisher;
	// service that returns the current temperature
	rclcpp::Service<GetCurrentTemperature>::SharedPtr getCurrentTemperatureService;
	// service that increments or decrements the current temperature
	rclcpp::Service<IncrementDecrementTemperature>::SharedPtr incrementDecrementTemperatureService;
	// action server to set the temperature
	rclcpp_action::Server<SetTemperature>::SharedPtr setTemperatureActionServer;
	// status of the action server busy or not
	volatile bool actionServerBusy = false;
	// thread to execute the action server
	thread actionThread;
	
	// uniform random number generator
	default_random_engine randomNumber;


public:
#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedValue"
	
	
	TemperatureSystemsControllerNode() : Node("temperature_control_systems") {
		// initialize the current temperature with random value between 15 and 100
		currentTemperature = (short) (15 + (randomNumber() % 85));
		// create a publisher
		temperaturePublisher = this->create_publisher<std_msgs::msg::Int16>(contextPrefix + "__temperature", 10);
		// create a monitor timer
		temperatureMonitorTimer = this->create_wall_timer(1s, [this] { temperature_monitor_callback(); });
		// create a service to return the current temperature
		getCurrentTemperatureService = this->create_service<GetCurrentTemperature>(
				contextPrefix + "__get_current_temperature",
				[this](const std::shared_ptr<GetCurrentTemperature::Request> &request,
				       const std::shared_ptr<GetCurrentTemperature::Response> &response) {
					get_current_temperature_callback(request, response);
				}
		);
		// create a service to increment or decrement the current temperature
		incrementDecrementTemperatureService = this->create_service<IncrementDecrementTemperature>(
				contextPrefix + "__increment_decrement_temperature",
				[this](const std::shared_ptr<IncrementDecrementTemperature::Request> &request,
				       const std::shared_ptr<IncrementDecrementTemperature::Response> &response) {
					increment_decrement_temperature_callback(request, response);
				}
		);
		
		// create an action server to set the temperature
		setTemperatureActionServer = rclcpp_action::create_server<SetTemperature>(
				this,
				contextPrefix + "__set_temperature",
				[this](const rclcpp_action::GoalUUID &uuid,
				       const shared_ptr<const SetTemperature::Goal> &goal) {
					return handle_set_temperature_action_callback(uuid, goal);
				},
				[this](const shared_ptr<rclcpp_action::ServerGoalHandle<SetTemperature>> &goalHandle) {
					return cancel_set_temperature_action_callback(goalHandle);
				},
				[this](const shared_ptr<rclcpp_action::ServerGoalHandle<SetTemperature>> &goalHandle) {
					accepted_set_temperature_action_callback(goalHandle);
				}
		);
		
	}

#pragma clang diagnostic pop

private:
	void
	temperature_monitor_callback() {
		// publish the current temperature
		auto message = std_msgs::msg::Int16();
		message.data = currentTemperature;
		RCLCPP_DEBUG(this->get_logger(), "Publishing: '%s'", to_string(message.data).c_str());
		temperaturePublisher->publish(message);
	}

private:
	void
	get_current_temperature_callback(
			const std::shared_ptr<GetCurrentTemperature::Request> &request,
			const std::shared_ptr<GetCurrentTemperature::Response> &response) {
		RCLCPP_INFO(this->get_logger(), "Incoming request for current temperature");
		response->temperature = currentTemperature;
	}

private:
	void
	increment_decrement_temperature_callback(
			const std::shared_ptr<IncrementDecrementTemperature::Request> &request,
			const std::shared_ptr<IncrementDecrementTemperature::Response> &response) {
		bool isIncrement = request->increment;
		RCLCPP_INFO(this->get_logger(), "Incoming request for %s temperature",
		            isIncrement ? "increment" : "decrement");
		// check if the temperature can be increased
		bool canIncreaseTemperature = randomNumber() % 2 == 0;
		if (!canIncreaseTemperature) {
			response->success = false;
			response->temperature = currentTemperature;
			response->message = "Temperature cannot be " + string(isIncrement ? "increased" : "decreased");
			return;
		}
		// sleep for a random time between 0.1s and 0.5s
		
		auto sleepTime = 100 + (randomNumber() % 400);
		this_thread::sleep_for(chrono::milliseconds(sleepTime));
		// increase/decrease the temperature
		currentTemperature += isIncrement ? 1 : -1;
		response->success = true;
		response->temperature = currentTemperature;
		response->message = "Temperature " + string(isIncrement ? "increased" : "decreased") + " successfully";
		RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
	}

private:
	rclcpp_action::GoalResponse
	handle_set_temperature_action_callback(const rclcpp_action::GoalUUID &uuid,
	                                       const shared_ptr<const SetTemperature::Goal> &goal) {
		RCLCPP_INFO(this->get_logger(), "Incoming request for setting temperature");
		if (actionServerBusy) {
			return rclcpp_action::GoalResponse::REJECT;
		}
		actionServerBusy = true;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

private:
	rclcpp_action::CancelResponse
	cancel_set_temperature_action_callback(
			const shared_ptr<rclcpp_action::ServerGoalHandle<SetTemperature>> &goalHandle) {
		RCLCPP_INFO(this->get_logger(), "Incoming request for cancelling setting temperature");
		return rclcpp_action::CancelResponse::ACCEPT;
	}

private:
	void
	accepted_set_temperature_action_callback(
			const shared_ptr<rclcpp_action::ServerGoalHandle<SetTemperature>> &goalHandle) {
		actionThread = thread([this, goalHandle]() {
			RCLCPP_INFO(this->get_logger(), "Incoming request for accepting setting temperature");
			auto feedback = std::make_shared<SetTemperature::Feedback>();
			auto result = std::make_shared<SetTemperature::Result>();
			auto goal = goalHandle->get_goal();
			auto targetTemperature = goal->temperature;
			auto initialTemperature = currentTemperature;
			auto temperature = currentTemperature;
			auto is_increment = targetTemperature > currentTemperature;
			
			while (temperature != targetTemperature) {
				// check if there is a cancel request
				if (goalHandle->is_canceling()) {
					result->success = false;
					result->temperature = temperature;
					result->message = "Setting temperature cancelled";
					goalHandle->canceled(result);
					actionServerBusy = false;
					RCLCPP_INFO(this->get_logger(), "Setting temperature cancelled");
					return;
				}
				
				// call the increment/decrement service
				auto request = std::make_shared<IncrementDecrementTemperature::Request>();
				request->increment = is_increment;
				// create a client for the service
				auto incrementDecrementTemperatureClient = this->create_client<IncrementDecrementTemperature>(
						contextPrefix + "__increment_decrement_temperature");
				// send the request
				auto future = incrementDecrementTemperatureClient->async_send_request(request);
				// wait for the response indefinitely
				auto response = future.get();
				// publish the feedback
				temperature = response->temperature;
				feedback->temperature = temperature;
				feedback->progress = (temperature - initialTemperature) * 100 /
				                     (targetTemperature - initialTemperature);
				goalHandle->publish_feedback(feedback);
				RCLCPP_DEBUG(this->get_logger(), "Publishing feedback: '%s'", to_string(feedback->progress).c_str());
			}
			
			result->success = true;
			result->temperature = temperature;
			result->message = "Setting temperature succeeded";
			goalHandle->succeed(result);
			actionServerBusy = false;
			RCLCPP_INFO(this->get_logger(), "Setting temperature succeeded");
		});
		
		actionThread.detach();
	}
	
};

int
main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TemperatureSystemsControllerNode>());
	rclcpp::shutdown();
	return 0;
}