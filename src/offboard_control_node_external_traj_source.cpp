#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <string>
#include <cstring>

using namespace std::chrono;
using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
		offboard_control_mode_publisher_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic", 1);
		trajectory_setpoint_publisher_ =
			this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 1);
		vehicle_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 1);

		// get FCU timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 1,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					if (msg->sys_id == 1)
					{
						timestamp_.store(msg->tc1);
					}
				});

		// update trajectory setpoint
		generated_traj_sp_sub_ =
			this->create_subscription<px4_msgs::msg::TrajectorySetpoint>("garrard_accelerationsetpoint_pubsub", 1,
				[this](const px4_msgs::msg::TrajectorySetpoint::UniquePtr sp) {
					traj_sp = *sp;

					RCLCPP_INFO(get_logger(), "Publishing trajectory setpoint");
					// offboard_control_mode needs to be paired with trajectory_setpoint
					publish_offboard_control_mode();
					publish_trajectory_setpoint();
				});

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_++ == 10) {
				RCLCPP_INFO(get_logger(), "Setting drone to offboard mode");
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				RCLCPP_INFO(get_logger(), "Arming drone");
				// Arm the vehicle
				this->arm();
			}
		};
		timer_ = this->create_wall_timer(500ms, timer_callback);
  }

	void arm() const;
	void disarm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;
  	rclcpp::TimerBase::SharedPtr traj_timer_;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr generated_traj_sp_sub_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

  px4_msgs::msg::TrajectorySetpoint traj_sp{};

	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	// msg.timestamp = timestamp_.load();
	msg.timestamp = 0;
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = true;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() const {
	// px4_msgs::msg::TrajectorySetpoint msg{};
	// msg.timestamp = timestamp_.load();
	// msg.x = 0.0;
	// msg.y = 0.0;
	// msg.z = -1.0;
	// msg.yaw = -3.14; // [-PI:PI]

	// trajectory_setpoint_publisher_->publish(msg);
  trajectory_setpoint_publisher_->publish(traj_sp);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	px4_msgs::msg::VehicleCommand msg{};
	//msg.timestamp = timestamp_.load();
	msg.timestamp = 0;
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
