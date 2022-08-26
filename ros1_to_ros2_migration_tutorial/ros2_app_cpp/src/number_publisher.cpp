#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
    public:
        NumberPublisherNode(): Node("number_publisher")
        {
            this->declare_parameter("number_to_publish", 2);
            this->declare_parameter("number_publish_frequency", 1.0);

            number_ = this->get_parameter("number_to_publish").as_int();
            double publish_frequency = this->get_parameter("number_publish_frequency").as_double();

            number_publisher_ = this->create_publisher<std_msgs::msg::Int64>("number", 10);

            number_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / publish_frequency)),
                                                    std::bind(&NumberPublisherNode::publishNumber, this));
            RCLCPP_INFO(this->get_logger(), "Number publisher has been started");
        }

    private:
        void printNumber()
        {
            RCLCPP_INFO(this->get_logger(), "%d", number_);
        }

        void publishNumber()
        {
            // auto msg = std_msgs::msg::Int64();
            std_msgs::msg::Int64 msg;
            msg.data = number_;
            number_publisher_->publish(msg);
        }

        int number_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr number_publisher_;
        rclcpp::TimerBase::SharedPtr number_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);  // init ROS 2 communication (no node created here)

    // create a node object, stored in a shared pointer
    // auto node = std::make_shared<rclcpp::Node>("number_publisher");  // basic way
    auto node = std::make_shared<NumberPublisherNode>();  // OOP way

    rclcpp::spin(node);

    rclcpp::shutdown();  // shutdown ROS 2 communication

    return 0;
}