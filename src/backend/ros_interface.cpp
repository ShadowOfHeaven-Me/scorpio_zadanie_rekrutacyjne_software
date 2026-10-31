#include "ros_interface.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>


namespace backend {

std::vector<std::pair<double, Point>> read_targets(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("cannot open input_file: " + path);
  }
  std::vector<std::pair<double, Point>> signals;
  std::string line;
  for (size_t number = 1; std::getline(file, line); ++number) {
    const auto where = path + ":" + std::to_string(number) + ": ";
    line.erase(std::min(line.find('#'), line.size()));
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;
    }
    std::istringstream stream(line);
    double delay;
    Point point;
    stream >> delay >> point.x >> point.y >> point.z;
    if (stream.fail() || !(stream >> std::ws).eof()) {
      throw std::runtime_error(where + "expected \"<delay> <x> <y> <z>\", got: " + line);
    }
    if (delay < 0.0) {
      throw std::runtime_error(where + "delay must be non-negative");
    }
    signals.emplace_back(delay, point);
  }
  return signals;
}

RosInterface::RosInterface(const rclcpp::NodeOptions& options)
: rclcpp::Node("interface", options) {
  const auto input_file = declare_parameter<std::string>("input_file", "");
  const auto encoder_period = declare_parameter<double>("encoder_period", 0.05);
  const auto debug = declare_parameter<bool>("debug", false);
  if (encoder_period <= 0.0) {
    throw std::runtime_error("encoder_period must be positive");
  }
  if (input_file.empty()) {
    throw std::runtime_error("the 'input_file' parameter is required");
  }
  auto signals = read_targets(input_file);

  // QoS is chosen so that the default rclcpp QoS (reliable, volatile) used by the solution
  // node is always compatible: publishers offer reliable delivery (a best-effort or reliable
  // subscription both match), subscriptions request volatile durability (a volatile or
  // transient-local publisher both match).
  _motor1_encoder = create_publisher<std_msgs::msg::UInt16>(
    "motor_1/encoder", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  _motor2_encoder = create_publisher<std_msgs::msg::UInt16>(
    "motor_2/encoder", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  _target = create_publisher<geometry_msgs::msg::Point>(
    "target", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

  // Threads are started only after every callback is attached, otherwise the first target
  // (delay 0) or the first encoder readings could be produced before anyone listens.
  _tester = std::make_shared<BasicTester>(encoder_period, std::move(signals), debug, Constraints{}, false);

  auto motor1_encoder = _motor1_encoder;
  _tester->get_motor_1()->add_data_callback([motor1_encoder](const uint16_t& value) {
    std_msgs::msg::UInt16 msg;
    msg.data = value;
    motor1_encoder->publish(msg);
  });
  auto motor2_encoder = _motor2_encoder;
  _tester->get_motor_2()->add_data_callback([motor2_encoder](const uint16_t& value) {
    std_msgs::msg::UInt16 msg;
    msg.data = value;
    motor2_encoder->publish(msg);
  });
  auto target = _target;
  _tester->get_commands()->add_data_callback([target](const Point& point) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x;
    msg.y = point.y;
    msg.z = point.z;
    target->publish(msg);
  });

  _motor1_control = subscribe_control("motor_1/control", _tester->get_motor_1());
  _motor2_control = subscribe_control("motor_2/control", _tester->get_motor_2());

  _tester->start();
}

rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr RosInterface::subscribe_control(
  const char* topic, std::shared_ptr<MotorComponent> motor) {
  return create_subscription<std_msgs::msg::Int8>(topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    [motor = std::move(motor)](const std_msgs::msg::Int8::SharedPtr msg) {
      motor->send_data(msg->data);
    });
}

}  // namespace backend

RCLCPP_COMPONENTS_REGISTER_NODE(backend::RosInterface)
