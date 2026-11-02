#include <cmath>
#include <algorithm>
#include <atomic>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace {
    // rotX
    constexpr double C1 = 10.0 / 180 * M_PI / (127 * 127);

    // rotY
    constexpr double C2 = 8.0 / 6 * 10.0 / 180 * M_PI / (128 * 128);

    constexpr int ENCODER_RESOLUTION = 4096;
    constexpr int UNCERTAINTY_BITS = 2;
} // namespace

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
            preempt = declare_parameter<bool>("preempt", false);
            encoder_period = declare_parameter<double>("encoder_period", 0.05);
            RCLCPP_INFO(get_logger(), "Mode: %s", preempt ? "Preempt" : "Queue");

            control_1 = create_publisher<std_msgs::msg::Int8>(
                "motor_1/control", rclcpp::QoS(10).reliable().transient_local());
            control_2 = create_publisher<std_msgs::msg::Int8>(
                "motor_2/control", rclcpp::QoS(10).reliable().transient_local());

            encoder_1 = create_subscription<std_msgs::msg::UInt16>(
                "motor_1/encoder",
                rclcpp::QoS(10).reliable().durability_volatile(),
                [this](const std_msgs::msg::UInt16::SharedPtr rotX) {
                    //if (target_reached) return;
                    const Angle target_angle = target.load();
                    const uint16_t target_bit = target_angle.rotXBit();
                    uint16_t x = rotX->data;

                    //delta od celu
                    int delta_target_bits = calc_shortest_delta(target_bit, x);
                    //delta ruchu
                    int delta_x = calc_shortest_delta(x, last_x);

                    std_msgs::msg::Int8 msg;

                    double delta_target_rad = static_cast<double>(delta_target_bits) / ENCODER_RESOLUTION * 2 * M_PI;
                    double velocity_rad = static_cast<double>(delta_x) / ENCODER_RESOLUTION * 2 * M_PI / encoder_period;

                    double Kp = 2;
                    double Kd = 0.4;
                    //przyspieszamy jak daleko, hamujemy jak blisko
                    double desired_velocity_rad = Kp * delta_target_rad - Kd * velocity_rad;
                    //speed = C * Bit^2
                    //Bit^2 = speed / C
                    //Bit = sqrt(speed / C)

                    double raw_bit = std::copysign(std::sqrt(std::abs(desired_velocity_rad) / C1),
                                                   desired_velocity_rad);
                    int clamped = std::clamp(static_cast<int>(std::round(raw_bit)), -128, 127);
                    msg.data = static_cast<int8_t>(clamped);

                    float f1 = static_cast<float>(last_x) / 4096 * 360;
                    float f2 = static_cast<float>(last_y) / 4096 * 360;

                    control_1->publish(msg);
                    //RCLCPP_INFO(get_logger(), "RotX: %.2f, RotY: %.2f, Bits X: %d Bits Y: %d", f1, f2, msg.data);
                    RCLCPP_INFO(get_logger(), "RotX: %.2f, RotY: %.2f, Target X: %.2f, Target Y: %.2f", f1, f2,
                                target_angle.rotX / M_PI * 180, target_angle.rotY / M_PI * 180);

                    last_x = x;
                    last_delta_x = delta_x;
                    check_if_target_reached(target_angle);
                });

            encoder_2 = create_subscription<std_msgs::msg::UInt16>(
                "motor_2/encoder",
                rclcpp::QoS(10).reliable().durability_volatile(),
                [this](const std_msgs::msg::UInt16::SharedPtr rotY) {
                    //if (target_reached) return;
                    const Angle target_angle = target.load();
                    const uint16_t target_bit = target_angle.rotYBit();
                    uint16_t y = rotY->data;

                    //delta od celu
                    int delta_target_bits = calc_shortest_delta(target_bit, y);
                    //delta ruchu
                    int delta_y = calc_shortest_delta(y, last_y);

                    std_msgs::msg::Int8 msg;

                    double delta_target_rad = static_cast<double>(delta_target_bits) / ENCODER_RESOLUTION * 2 * M_PI;
                    double velocity_rad = static_cast<double>(delta_y) / ENCODER_RESOLUTION * 2 * M_PI / encoder_period;

                    double Kp = 2;
                    double Kd = 0.4;
                    //przyspieszamy jak daleko, hamujemy jak blisko
                    double desired_velocity_rad = Kp * delta_target_rad - Kd * velocity_rad;
                    //speed = C * Bit^2
                    //Bit^2 = speed / C
                    //Bit = sqrt(speed / C)

                    double raw_bit = std::copysign(std::sqrt(std::abs(desired_velocity_rad) / C1),
                                                   desired_velocity_rad);
                    int clamped = std::clamp(static_cast<int>(std::round(raw_bit)), -128, 127);
                    msg.data = static_cast<int8_t>(clamped);

                    //float f = static_cast<float>(y) / 4096 * 360;

                    control_2->publish(msg);
                    //RCLCPP_INFO(get_logger(), "RotY: %.2f, Bits: %d", f, msg.data);

                    last_y = y;
                    last_delta_y = delta_y;
                    check_if_target_reached(target_angle);
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

                    auto angle = Angle{alpha, beta};
                    if (preempt || target_reached) {
                        target.store(angle);
                        target_reached = false;
                    } else {
                        target_list.push_back(angle);
                    }
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

        void check_if_target_reached(const Angle &target_angle) {
            if (target_reached) return;
            bool reachedX = std::abs(calc_shortest_delta(last_x, target_angle.rotXBit())) <= UNCERTAINTY_BITS;
            // && std::abs(last_delta_x) <= UNCERTAINTY_BITS;
            bool reachedY = std::abs(calc_shortest_delta(last_y, target_angle.rotYBit())) <= UNCERTAINTY_BITS;
            // && std::abs(last_delta_y) <= UNCERTAINTY_BITS;

            if (!reachedX || !reachedY) return;

            RCLCPP_INFO(get_logger(), "TARGET REACHED X=%u Y=%u T_x=%u T_y=%u", target_angle.rotXBit(),
                        target_angle.rotYBit(), last_x, last_y);
            //RCLCPP_INFO(get_logger(), "TARGET REACHED X=%.2f Y=%.2f", target_angle.rotX * 180 / M_PI, target_angle.rotY * 180 / M_PI);


            if (target_list.empty()) {
                target_reached = true;
                return;
            }

            auto new_angle = target_list.front();
            target_list.pop_front();
            target.store(new_angle);
        }

        bool preempt;
        double encoder_period;
        volatile bool target_reached = true;
        std::atomic<Angle> target{Angle{0.0, 0.0}};
        volatile u_int16_t last_x = 0, last_y = 0;
        volatile int last_delta_x = 0, last_delta_y = 0;
        std::deque<Angle> target_list; //dbr uznajmy że to jest thread-safe
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr control_1;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr control_2;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr encoder_1;
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr encoder_2;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_reader;
    };
} // namespace solution

RCLCPP_COMPONENTS_REGISTER_NODE(solution::Solution)
