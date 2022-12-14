#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "my_robot_interfaces/srv/reset_counter.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_ (0)
    {
        number_subscriber_ = this->create_subscription<std_msgs::msg::Int64>
                                ("number", 10, std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));

        reset_counter_service_ = this->create_service<my_robot_interfaces::srv::ResetCounter>
                                ("reset_counter", std::bind(&NumberCounterNode::callbackResetCounter, this,
                                 std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Number counter node has been started");
    }

private:
    void callbackNumber(const std_msgs::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        RCLCPP_INFO(get_logger(), "Counter: %d", counter_);
    }

    void callbackResetCounter(const my_robot_interfaces::srv::ResetCounter::Request::SharedPtr request,
                              const my_robot_interfaces::srv::ResetCounter::Response::SharedPtr response)
    {
        if (request->reset_value >= 0)
        {
            counter_ = request->reset_value;
            RCLCPP_INFO(get_logger(), "Counter: %d", counter_);
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }

    int counter_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr number_subscriber_;
    rclcpp::Service<my_robot_interfaces::srv::ResetCounter>::SharedPtr reset_counter_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}