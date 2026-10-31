// You can remove or add any includes you need
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
//#include <std_msgs/msg/detail/u_int16__struct.hpp>

namespace solution {

class Solution final : public rclcpp::Node {
public:
  explicit Solution(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("solution", options) {
    // Short example you can remove it
    const auto preempt = declare_parameter<bool>("preempt", false);
    RCLCPP_INFO(get_logger(), "%s", preempt ? "Preempt" : "Queue");

    control_1 = create_publisher<std_msgs::msg::Int8>("motor_1/control", rclcpp::QoS(10).reliable().transient_local());
    //_motor2 = create_publisher<std_msgs::msg::Int8>("motor_2/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());


    encoder_1 = create_subscription<std_msgs::msg::UInt16>("motor_1/encoder", rclcpp::QoS(10).reliable().durability_volatile(),
      [this](const std_msgs::msg::UInt16::SharedPtr rotX) {
        RCLCPP_INFO(get_logger(), "Rot: %u", rotX->data);
      });

    /*_target = create_subscription<geometry_msgs::msg::Point>("target", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const geometry_msgs::msg::Point::SharedPtr point) {
        RCLCPP_INFO(get_logger(), "Target: (%g, %g, %g)", point->x, point->y, point->z);

        std_msgs::msg::Int8 msg;
        msg.data = 100;
        _motor1->publish(msg);
      });*/
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr control_1;
  //rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor2;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr encoder_1;
  //rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _target;
};

}  // namespace solution

RCLCPP_COMPONENTS_REGISTER_NODE(solution::Solution)
