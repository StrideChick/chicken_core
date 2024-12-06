#include "joy_translate/joy_translate.hpp"
#include "joy_translate/logcool_F710.hpp"

JoyTranslate::JoyTranslate()
: Node("joy_translate_node")
{
    joy_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("manual_cmd_vel", 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("mode", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&JoyTranslate::joy_output_cb, this, std::placeholders::_1));
}

void JoyTranslate::joy_output_cb(const sensor_msgs::msg::Joy &msg)
{
    controller.update_key_value(msg);

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = controller.joy_left_y > 0 ? 
                     controller.joy_left_y * max_linear_velocity_ : 
                     std::abs(controller.joy_left_y) * min_linear_velocity_;

    twist_msg.angular.z = controller.joy_right_x > 0 ? 
                     controller.joy_right_x * max_angular_velocity_ : 
                     std::abs(controller.joy_right_x) * min_angular_velocity_;
    joy_twist_pub_->publish(twist_msg);

    auto mode_msg = std_msgs::msg::Int32();
    // if (controller.A_btn) {
    //     mode = 1;
    // } else if (controller.B_btn) {
    //     mode = 2;
    // } else if (controller.Y_btn) {
    //     mode = 3;
    // } else if (controller.X_btn) {
    //     mode = 4;
    // }
    if (controller.B_btn) {
        mode = 2;
    } else if (controller.Y_btn) {
        mode = 3;
    } else if (controller.X_btn) {
        mode = 4;
    }
    mode_msg.data = mode;
    mode_pub_->publish(mode_msg);

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyTranslate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
