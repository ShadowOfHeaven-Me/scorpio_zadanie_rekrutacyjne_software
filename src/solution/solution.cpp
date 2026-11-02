#include <cmath>
#include <algorithm>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace {
// rotX
constexpr double C1 = 10.0 / 180 * M_PI / (127 * 127);

// rotY
constexpr double C2 = 8.0/6 * 10.0 / 180 * M_PI / (128 * 128);

constexpr int ENCODER_RESOLUTION = 4096;
constexpr int UNCERTAINTY_BITS = 2;
}  // namespace

struct Angle {
    // w radianach, [0, 2pi)
    double rotX{0.0};
    double rotY{0.0};

    uint16_t rotXBit() const {
        return static_cast<uint16_t>((rotX / (2.0 * M_PI)) * ENCODER_RESOLUTION);
    }

    uint16_t rotYBit() const {
        return static_cast<uint16_t>((rotY / (2.0 * M_PI)) * ENCODER_RESOLUTION);
    }
};

namespace solution {

class Solution final : public rclcpp::Node {
public:
    explicit Solution(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp::Node("solution", options) {
        const auto preempt = declare_parameter<bool>("preempt", false);
        RCLCPP_INFO(get_logger(), "Mode: %s", preempt ? "Preempt" : "Queue");

        control_1 = create_publisher<std_msgs::msg::Int8>(
            "motor_1/control", rclcpp::QoS(10).reliable().transient_local());
        control_2 = create_publisher<std_msgs::msg::Int8>(
            "motor_2/control", rclcpp::QoS(10).reliable().transient_local());

        encoder_1 = create_subscription<std_msgs::msg::UInt16>(
            "motor_1/encoder",
            rclcpp::QoS(10).reliable().durability_volatile(),
            [this](const std_msgs::msg::UInt16::SharedPtr rotX) {
                const Angle target_angle = target.load();
                const uint16_t target_bit = target_angle.rotXBit();
                uint16_t x = rotX->data;

                //delta od celu
                int delta_target_bits = calc_shortest_delta(target_bit, x);
                //delta ruchu
                int delta_x = calc_shortest_delta(x, last_x);

                std_msgs::msg::Int8 msg;

                double Kp = 2;
                double Kd = 0.8;
                //przyspieszamy jak daleko, hamujemy jak blisko
                double desired_velocity_bits = Kp * delta_target_bits - Kd * delta_x;
                double desired_velocity_rad = desired_velocity_bits / ENCODER_RESOLUTION * 2 * M_PI;
                //speed = C * Bit^2
                //Bit^2 = speed / C
                //Bit = sqrt(speed / C)

                double raw_bit = std::sqrt(desired_velocity_rad / C1);
                int clamped = std::clamp(static_cast<int>(std::round(raw_bit)), -128, 127);
                msg.data = static_cast<int8_t>(clamped);

                /*if (std::abs(delta_bits) <= UNCERTAINTY_BITS) {
                    msg.data = 0;
                } else {
                    //10 stopni/s * 0.5s = 5 stopni
                    double path_deg = (static_cast<double>(delta_bits) / ENCODER_RESOLUTION) * 360;
                    int bits_speed;

                    //taką wartość drogi hamowania znalazłem max -> 0
                    if (std::abs(path_deg) >= 2.1) {
                        bits_speed = path_deg < 0 ? -128 : 127;
                    } else {
                        
                        bits_speed = 0;
                    }

                    msg.data = static_cast<int8_t>(bits_speed);
                }*/

                //float f = static_cast<float>(x) / 4096 * 360;

                control_1->publish(msg);
                //RCLCPP_INFO(get_logger(), "RotX: %.2f, Bits: %d", f, msg.data);

                last_x = x;
            });

        encoder_2 = create_subscription<std_msgs::msg::UInt16>(
            "motor_2/encoder",
            rclcpp::QoS(10).reliable().durability_volatile(),
            [this](const std_msgs::msg::UInt16::SharedPtr rotY) {
                const Angle target_angle = target.load();
                 const uint16_t target_bit = target_angle.rotYBit();
                 uint16_t y = rotY->data;

                 //delta od celu
                 int delta_target_bits = calc_shortest_delta(target_bit, y);
                 //delta ruchu
                 int delta_y = calc_shortest_delta(y, last_y);

                 std_msgs::msg::Int8 msg;

                 double Kp = 2;
                 double Kd = 0.8;
                 //przyspieszamy jak daleko, hamujemy jak blisko
                 double desired_velocity_bits = Kp * delta_target_bits - Kd * delta_y;
                 double desired_velocity_rad = desired_velocity_bits / ENCODER_RESOLUTION * 2 * M_PI;
                 //speed = C * Bit^2
                 //Bit^2 = speed / C
                 //Bit = sqrt(speed / C)

                 double raw_bit = std::sqrt(desired_velocity_rad / C2);
                 int clamped = std::clamp(static_cast<int>(std::round(raw_bit)), -128, 127);
                 msg.data = static_cast<int8_t>(clamped);

                 float f = static_cast<float>(y) / 4096 * 360;

                 control_2->publish(msg);
                 RCLCPP_INFO(get_logger(), "RotY: %.2f, Bits: %d", f, msg.data);

                 last_y = y;
            });

        target_reader = create_subscription<geometry_msgs::msg::Point>(
            "target", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
            [this](const geometry_msgs::msg::Point::SharedPtr point) {

                //-y bo idzie od +X do -y
                double alpha = std::atan2(-point->y, point->x);
                if (alpha < 0)
                    alpha += 2.0 * M_PI;

                double R = std::hypot(point->x, point->y);
                double beta = std::atan2(point->z, R);
                if (beta < 0)
                    beta += 2.0 * M_PI;

                target.store(Angle{alpha, beta});
                RCLCPP_INFO(get_logger(), "Target, rotX=%.2f, rotY=%.2f", alpha * 180 / M_PI, beta * 180 / M_PI);
            });
    }

private:
    static int calc_shortest_delta(uint16_t target_bit, uint16_t current_bit) {
        int delta = static_cast<int>(target_bit) - static_cast<int>(current_bit);
        while (delta > ENCODER_RESOLUTION / 2) delta -= ENCODER_RESOLUTION;
        while (delta < -ENCODER_RESOLUTION / 2) delta += ENCODER_RESOLUTION;
        return delta;
    }

    std::atomic<Angle> target{Angle{0.0, 0.0}};
    volatile u_int16_t last_x = 0, last_y = 0;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr control_1;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr control_2;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr encoder_1;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr encoder_2;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_reader;
};

} // namespace solution

RCLCPP_COMPONENTS_REGISTER_NODE(solution::Solution)