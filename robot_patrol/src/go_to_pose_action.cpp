#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class GoToPose : public rclcpp::Node
{
public:
    using GoToPoseAction = robot_patrol::action::GoToPose;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

    explicit GoToPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("go_to_pose_node", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<GoToPoseAction>
        (
            this,
            "go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER READY");

        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>
        (
            "/odom", 
            10,
            std::bind
            (
                        &GoToPose::odomCallback, 
                        this, 
                        std::placeholders::_1
            )
        );

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>
                (
                "/cmd_vel", 
                5
                );
        timer_ = this->create_wall_timer
                (
                std::chrono::milliseconds(100), 
                std::bind(&GoToPose::timer_callback, this)
                );
    }

private:
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool target_achieved = false;

    double goal_x = 0.0;
    double goal_y = 0.0;
    double goal_theta = 0.0;

    double current_x = 0.0;
    double current_y = 0.0;
    double current_theta = 0.0;

    double dx = 0.0;
    double dy = 0.0;
    double dtheta = 0.0;

    double angle_from_front = 0.0;

    double angular_z_vel = 0.0;
    double linear_x_vel = 0.0;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const GoToPoseAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER GOAL RECEIVED");
        (void)uuid;
        this->goal_x = goal->goal_pos.x;
        this->goal_y = goal->goal_pos.y;
        this->goal_theta = goal->goal_pos.theta;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER GOAL CANCELLED");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER RUNNING GOAL");

        this->target_achieved = false;
        
        auto feedback = std::make_shared<GoToPoseAction::Feedback>();
        auto result = std::make_shared<GoToPoseAction::Result>();

        rclcpp::Rate loop_rate(10); // 10 Hz

        this->dtheta = normalize_angle(this->goal_theta - this->current_theta);
        double angle_error = fabs(this->dtheta);
        while(angle_error > 0.1)
        {
            if (goal_handle->is_canceling())
            {
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "ACTION SERVER CANCELLING GOAL");
                return;
            }

            if(this->dtheta > 0.0)
            {
                this->angular_z_vel = 0.1;
            }
            else
            {
                this->angular_z_vel = -0.1;
            }
            this->dtheta = normalize_angle(this->goal_theta - this->current_theta);
            angle_error = fabs(this->dtheta);
            loop_rate.sleep();
        }
        this->angular_z_vel = 0.0;
        
        while(!this->target_achieved)
        {
            if (goal_handle->is_canceling())
            {
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "ACTION SERVER CANCELLING GOAL");
                return;
            }

            this->dx = this->goal_x - this->current_x;
            this->dy = this->goal_y - this->current_y;

            this->linear_x_vel = 0.2;
            // this->angle_from_front = (3.1415926535 / 2) - atan2(this->dy, this->dx);
            this->angle_from_front = atan2(this->dy, this->dx);
            this->angular_z_vel = this->angle_from_front / 2; // From section 1: angular_z_vel_ = direction_ / 2
            
            feedback->current_pos.x = current_x;
            feedback->current_pos.y = current_y;
            feedback->current_pos.theta = current_theta;
            goal_handle->publish_feedback(feedback);

            double distance_error = sqrt(this->dx * this->dx + this->dy * this->dy);
            if (distance_error < 0.1)
            {
                this->target_achieved = true;
            }
            loop_rate.sleep();
        }
        this->angular_z_vel = 0.0;
        this->linear_x_vel = 0.0;

        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER GOAL REACHED");
        return;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->current_x = msg->pose.pose.position.x;
        this->current_y = msg->pose.pose.position.y;

        // Convert quaternion to yaw (theta)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, this->current_theta);
    }

    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = this->linear_x_vel;
        msg.angular.z = this->angular_z_vel;
        this->publisher_->publish(msg);
    }

    // Fixing a bug with a common script
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
}; // CLASS GoToPose


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
} // MAIN
