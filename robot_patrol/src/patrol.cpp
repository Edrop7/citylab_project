#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <map>

// 2. CREATE A C++ CLASS NAMED Patrol
class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node") // CONSTRUCTOR
    {
        // separate publisher and subscriber into threads using callback groups and executors
        cb_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_pub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 3. GET THE LASER DATA
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_sub_;
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            (
            "/scan", 
            5,
            std::bind(&Patrol::callback_laser, this, std::placeholders::_1),
            sub_options
            );


        // 6. CREATE A CONTROL LOOP OF 10 HZ
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>
            (
            "/cmd_vel", 
            5
            );
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz -> run every 100 ms (10 times a second)
            std::bind(&Patrol::callback_velocity, this), // callback for publisher separated
            cb_group_pub_
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::CallbackGroup::SharedPtr cb_group_sub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_pub_;

    std::vector<float> front_laser_scan;

    float linear_x_vel_ = 0.1; // THE LINEAR VELOCITY IS ALWAYS 0.1 m/s 
    float angular_z_vel_ = 0.0;

    float direction_ = 0.0;
    float min_frontal_distance_allowed_ = 0.4; // in m, 40 cm
    float min_diagonal_distance_allowed_ = 0.2; // in m, 20 cm

    void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 4. GET THE RAYS CORRESPONDING TO THE FRONTAL 180 DEGREES
        this->front_laser_scan = std::vector<float>(msg->ranges.begin() + 179, msg->ranges.begin() + 540);

        stateflow_navigator(this->front_laser_scan);
    }

    // 5. ALGORITHM TO IMPLEMENT
    void stateflow_navigator(std::vector<float> &front_laser_scan)
    {
        float max_distance = 0.0;
        float distance_of_index = 0.0;
        int max_index = 0;

        // 5. MOVE THE ROBOT FORWARD UNTIL AN OBSTACLE IN FRONT OF ROBOT IS CLOSER THAN 35 CM
        if (front_laser_scan[179] > this->min_frontal_distance_allowed_)
        {
            // front-right check to fix bug with counter-clockwise lap
            if(front_laser_scan[89] < this->min_diagonal_distance_allowed_)
            {
                this->angular_z_vel_ = 3.141592 / 8;
                RCLCPP_INFO(this->get_logger(), "ADJUSTMENT FOR FRONT-RIGHT OBSTACLE WITHIN: %f",
                                            front_laser_scan[89]);
            }
            else if(front_laser_scan[269] < this->min_diagonal_distance_allowed_)
            {
                this->angular_z_vel_ = -3.141592 / 8;
                RCLCPP_INFO(this->get_logger(), "ADJUSTMENT FOR FRONT-RIGHT OBSTACLE WITHIN: %f",
                                            front_laser_scan[89]);
            } 
            else 
            {
                this->angular_z_vel_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "No frontal obstacle, cm to next obstacle: %f",
                                            front_laser_scan[179]);
            }
        }
        else if (front_laser_scan[179] < this->min_frontal_distance_allowed_)
        {
            // 5. DETERMINE WHICH RAY IN FRONTAL 180 IS THE LARGEST BUT NOT INFINITE
            for (int index = 0; index < 360; index++) {
                distance_of_index = front_laser_scan[index];
                if(std::isfinite(distance_of_index) && distance_of_index > max_distance) {
                    max_distance = distance_of_index;
                    max_index = index;
                }
            }

            // 4. & 5. DETERMINE WHAT IS THE SAFEST DIRECTION GIVEN THE MAX INDEX (radians)
            this->direction_ = (-3.141592/2) + ((max_index+1) * (3.141592/360));

            // 5. ANGULAR VELOCITY = DIRECTION (radians) / 2 (rad/s)
            // this->angular_z_vel_ = this->direction_ / 2;
            if (this->direction_ < 0.0) 
            {
                this->angular_z_vel_ = -3.141592 / 4;
            }
            else 
            {
                this->angular_z_vel_ = 3.141592 / 4;
            }
            
            RCLCPP_INFO(this->get_logger(), "OBSTACLE DETECTED WITHIN: %f cm;   VELOCITY SENT: %f rad/s",
                                            front_laser_scan[179],
                                            this->angular_z_vel_);
        }
    }

    // 6. CREATE A CONTROL LOOP OF 10 HZ (see constructor)
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

    // Concurrency with Mutually Exclusive MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}