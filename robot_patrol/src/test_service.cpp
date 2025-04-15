// 1. create a C++ file to test the service
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>

using GetDirection = custom_interfaces::srv::GetDirection;
using namespace std::chrono_literals;
using std::placeholders::_1;

// 2. create a simple node
class DirectionClient : public rclcpp::Node
{
public:
    DirectionClient() : Node("client_direction_node")
    {
        client_ = this->create_client<GetDirection>("direction_service");

        // 2. subscribes to the laser data
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DirectionClient::laser_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "CLIENT /direction_service READY");

        // 2. in a callback call /direction_service with thee data
        timer_ = this->create_wall_timer(
            500ms, std::bind(&DirectionClient::timer_callback, this));
    }

private:
    rclcpp::Client<GetDirection>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::LaserScan::SharedPtr laser_reading_stored_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        this->laser_reading_stored_ = msg;
    }

    void timer_callback()
    {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *(this->laser_reading_stored_);

        RCLCPP_INFO(this->get_logger(), "CLIENT /direction_service REQUEST SENT");

        auto future = client_->async_send_request(request, [this](rclcpp::Client<GetDirection>::SharedFuture result)
        {  
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "CLIENT /direction_service RESPONSE RECEIVED \n Direction: %s", response->direction.c_str());
            rclcpp::shutdown();
        });

        timer_->cancel();
    }
}; // CLASS DirectionClient

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionClient>());
    return 0;
} // MAIN