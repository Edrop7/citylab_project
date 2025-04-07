#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>

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
            10, // HZ
            std::bind(&Patrol::callback_laser, this, std::placeholders::_1)
            );

        // 6. CREATE A CONTROL LOOP OF 10 HZ
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>
            (
            "/cmd_vel", 
            10 // HZ
            );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

    std::vector<float> front_laser_scan;
    std::map<std::string, float> laser_reading_map;

    float linear_x_vel_ = 0.1; // THE LINEAR VELOCITY IS ALWAYS 0.1 m/s 
    float angular_z_vel_ = 0.0;

    float direction_ = 0.0;

    void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 4. GET THE RAYS CORRESPONDING TO THE FRONTAL 180 DEGREES
        this->front_laser_scan = std::vector<float>(msg->ranges.begin() + 179, msg->ranges.begin() + 540);

        this->laser_reading_map["front"] = msg->ranges[359];

        RCLCPP_INFO(this->get_logger(), "front distance: %.2f", this->laser_reading_map["front"]);

        stateflow_navigator(this->laser_reading_map, this->front_laser_scan);
        publish_velocities();
    }

    // 5. ALGORITHM TO IMPLEMENT
    void stateflow_navigator(const std::map<std::string, float> &laser_reading_map, std::vector<float> &front_laser_scan)
    {
        float max_distance = 0.0;
        float distance_of_index = 0.0;
        int max_index = 0;

        // 5. MOVE THE ROBOT FORWARD UNTIL AN OBSTACLE IN FRONT OF ROBOT IS CLOSER THAN 35 CM
        if (laser_reading_map.at("front") > 0.35)
        {
            this->angular_z_vel_ = 0.0;
        }
        else if (laser_reading_map.at("front") < 0.35)
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
            this->direction_ = -(3.141592/2) + (max_index * (3.141592/180)); 

            // 5. ANGULAR VELOCITY = DIRECTION (radians) / 2 (rad/s)
            this->angular_z_vel_ = this->direction_ / 2;
        }
    }
    // 6. CREATE A CONTROL LOOP OF 10 HZ (SEE CONSTRUCTOR)
    void publish_velocities()
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