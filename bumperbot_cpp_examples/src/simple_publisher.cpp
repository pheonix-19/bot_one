#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<chrono>

using namespace std::chrono_literals;
class SimplePublisher : public rclcpp::Node
{
    public:
        SimplePublisher() : Node("simple_publisher"), counter_(0)
        {
            pub_ = create_publisher<std_msgs::msg::String>("chatter",10);
            timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this)); 

            RCLCPP_INFO(get_logger(), "Simple Publisher has been started.");
        }
    private:
        unsigned int counter_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;


        void timerCallback()
        {
            auto msg = std_msgs::msg::String();
            msg.data = "Hello Ayush: " + std::to_string(counter_++);
            RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
            pub_->publish(msg);
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}