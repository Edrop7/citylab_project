#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <map>

// 2. CREATE A C++ CLASS
class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node") // CONSTRUCTOR
    {
        // 3. GET THE LASER DATA
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            (
            "/scan", 
            10,
            std::bind(&Patrol::callback_laser, this, std::placeholders::_1)
            );

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>
            (
            "/cmd_vel", 
            10
            );

        // 6. CREATE A CONTROL LOOP OF 10 HZ
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz -> run every 100 ms (10 times a second)
            std::bind(&Patrol::callback_velocity, this) // callback for publisher separated
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<float> front_laser_scan;
    std::map<std::string, float> laser_reading_map;

    float linear_x_vel_ = 0.1; // THE LINEAR VELOCITY IS ALWAYS 0.1 m/s 
    float angular_z_vel_ = 0.0;

    float direction_ = 0.0;
    float min_distance_allowed_ = 0.35; //in m

    void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 4. GET THE RAYS CORRESPONDING TO THE FRONTAL 180 DEGREES
        this->front_laser_scan = std::vector<float>(msg->ranges.begin() + 179, msg->ranges.begin() + 540);

        this->laser_reading_map["front"] = msg->ranges[359];

        stateflow_navigator(this->laser_reading_map, this->front_laser_scan);
    }

    // 5. ALGORITHM TO IMPLEMENT
    void stateflow_navigator(const std::map<std::string, float> &laser_reading_map, std::vector<float> &front_laser_scan)
    {
        float max_distance = 0.0;
        float distance_of_index = 0.0;
        int max_index = 0;

        // 5. MOVE THE ROBOT FORWARD UNTIL AN OBSTACLE IN FRONT OF ROBOT IS CLOSER THAN 35 CM
        if (laser_reading_map.at("front") > this->min_distance_allowed_)
        {
            this->angular_z_vel_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "No frontal obstacle, cm to next obstacle: %f",
                                            laser_reading_map.at("front"));
        }
        else if (laser_reading_map.at("front") < this->min_distance_allowed_)
        {
            // 5. DETERMINE WHICH RAY IN FRONTAL 180 IS THE LARGEST BUT NOT INFINITE
            for (int index = 0; index < 360; index++) {
                distance_of_index = front_laser_scan[index];
                if(std::isfinite(distance_of_index) && distance_of_index > max_distance) {
                    max_distance = distance_of_index;
                    max_index = index;
                }
            }

            this->laser_reading_map["max"] = max_distance; // in case I want to log it

            // 4. & 5. DETERMINE WHAT IS THE SAFEST DIRECTION GIVEN THE MAX INDEX (radians)
            this->direction_ = (-3.141592/2) + (max_index * (3.141592/180));

            // 5. ANGULAR VELOCITY = DIRECTION (radians) / 2 (rad/s)
            this->angular_z_vel_ = this->direction_ / 2;

            RCLCPP_INFO(this->get_logger(), "OBSTACLE DETECTED WITHIN: %f cm;   VELOCITY SENT: %f rad/s",
                                            laser_reading_map.at("front"),
                                            this->angular_z_vel_);
        }
    }

    // 6. CREATE A CONTROL LOOP OF 10 HZ
    void callback_velocity()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = this->linear_x_vel_;
        msg.angular.z = this->angular_z_vel_;
        this->pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}