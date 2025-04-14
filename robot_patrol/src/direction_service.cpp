#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <memory>
#include <algorithm>

using GetDirection = custom_interfaces::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node
{
public:
    DirectionService() : Node("server_direction_service_node")
    {
        srv_ = create_service<GetDirection>("direction_service", 
                                            std::bind(&DirectionService::request_callback, 
                                                        this, 
                                                        _1, 
                                                        _2
                                                    )
                                            );

        RCLCPP_INFO(this->get_logger(), "SERVICE /direction_service READY");
    }

private:
    rclcpp::Service<GetDirection>::SharedPtr srv_;

    void request_callback
    (
        const std::shared_ptr<GetDirection::Request> request,
        const std::shared_ptr<GetDirection::Response> response
    ) 
    {
        RCLCPP_INFO(this->get_logger(), "SERVICE /direction_service REQUESTED");

        // GET THE RAYS CORRESPONDING TO THE THREE SECTIONS (RIGHT, FRONT, LEFT)
        std::vector<float> left_laser_scan;
        std::vector<float> front_laser_scan;
        std::vector<float> right_laser_scan;
        right_laser_scan = std::vector<float>(request->laser_data.ranges.begin() + 179, request->laser_data.ranges.begin() + 300);
        left_laser_scan = std::vector<float>(request->laser_data.ranges.begin() + 419, request->laser_data.ranges.begin() + 540);
        // front scan is a different range to modify turning precision (maybe the lidar readings arent ideally split?)
        front_laser_scan = std::vector<float>(request->laser_data.ranges.begin() + 334, request->laser_data.ranges.begin() + 420);

        // FIND THE SUM OF EACH DIRECTION
        float left_sum = 0.0;
        float right_sum = 0.0;
        float left_reading = 0.0;
        float right_reading = 0.0;
        for (int i = 0; i < 120; i++)
        {
            /*
            ----------------------------------------
            I WILL CONVERT "inf" READINGS to 3.5 m
            ----------------------------------------
            FOUND IN
            --------
            simulation_ws/src/turtlebot3/turtlebot3_lds_2d.lua
            min_range = 0.12
            max_range = 3.5
            Laser readings range from 12 cm to 3.5 m
            */
            if(std::isfinite(left_laser_scan[i])) {
                left_reading = left_laser_scan[i];
            } else {
                left_reading = 3.5;
            }
            if(std::isfinite(right_laser_scan[i])) {
                right_reading = right_laser_scan[i];
            } else {
                right_reading = 3.5;
            }

            left_sum += left_reading;
            right_sum += right_reading;
        }

        // DETERMINE DIRECTION
        float min_front_value = *std::min_element(front_laser_scan.begin(), front_laser_scan.end());
        if (min_front_value > 0.35) // minimum front distance must be above 35 cm to not turn
        {
            response->direction = "forward";
        }
        else 
        {
            if (left_sum > right_sum)
            {
                response->direction = "left";
            }
            else if (right_sum > left_sum)
            {
                response->direction = "right";
            }
            else
            {
                response->direction = "error";
            }
        }

        RCLCPP_INFO(this->get_logger(), "SERVICE /direction_service COMPLETED");
        
        // USED IN DEBUGGING AN ISSUE
        // RCLCPP_INFO(this->get_logger(), "left: %f,    right: %f,    front: %f", left_sum, right_sum, front_sum);
    }

}; // CLASS DirectionService

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
} // MAIN