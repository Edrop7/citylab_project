// 1. Create a duplicate of patrol.cpp with new name
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>

using GetDirection = custom_interfaces::srv::GetDirection;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node") // CONSTRUCTOR
    {
        // separate publisher and subscriber into threads using callback groups and executors
        cb_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_pub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 2. add a service client that connects to /direction_service
        client_ = this->create_client<GetDirection>("direction_service");

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_sub_;
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            (
            "/scan", 
            10, 
            std::bind(&Patrol::laser_callback, this, _1),
            sub_options
            );
        RCLCPP_INFO(this->get_logger(), "PATROL CLIENT /direction_service READY");

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>
            (
            "/cmd_vel", 
            5
            );
        timer_ = this->create_wall_timer
            (
            100ms, 
            std::bind(&Patrol::timer_callback, this),
            cb_group_pub_
            );
    }

private:
    rclcpp::Client<GetDirection>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    rclcpp::CallbackGroup::SharedPtr cb_group_sub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_pub_;

    sensor_msgs::msg::LaserScan::SharedPtr laser_reading_stored_;

    float linear_x_vel_ = 0.0;
    float angular_z_vel_ = 0.0;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        this->laser_reading_stored_ = msg;
    }

    // 3. call the service to get the next direction to move
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
        if (laser_reading_stored_) {
            request->laser_data = *(this->laser_reading_stored_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Laser scan data not available.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "CLIENT /direction_service REQUEST SENT");

        auto future = client_->async_send_request(request, [this](rclcpp::Client<GetDirection>::SharedFuture result)
        {  
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "CLIENT /direction_service RESPONSE RECEIVED \n Direction: %s", response->direction.c_str());
            stateflow_navigator(response->direction);
            callback_velocity();
        });
    }

    // 3. assign speeds to the robot 
    void stateflow_navigator(std::string direction_response)
    {
        if (direction_response == "right")
        {
            this->linear_x_vel_ = 0.05;
            this->angular_z_vel_ = -0.5;
        }
        else if (direction_response == "left")
        {
            this->linear_x_vel_ = 0.05;
            this->angular_z_vel_ = 0.5;
        }
        else if (direction_response == "forward")
        {
            this->linear_x_vel_ = 0.1;
            this->angular_z_vel_ = 0.0;
        }
        else
        {
            this->linear_x_vel_ = 0.0;
            this->angular_z_vel_ = 0.0;
        }
    }

    void callback_velocity()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = this->linear_x_vel_;
        msg.angular.z = this->angular_z_vel_;
        this->pub_->publish(msg);
    }
}; // CLASS Patrol

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    // Concurrency with Mutually Exclusive MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
} // MAIN