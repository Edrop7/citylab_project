// 1. Create C++ file
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/callback_group.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

// 2. Create C++ Class
class GoToPose : public rclcpp::Node
{
public:
    // 7. use a custom interface
    using GoToPoseAction = robot_patrol::action::GoToPose;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

    explicit GoToPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("go_to_pose_node", options)
    {
        using namespace std::placeholders;

        // separate publisher and subscriber into threads using callback groups and executors
        cb_group_sub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_pub_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 3. Create the action server /go_to_pose
        this->action_server_ = rclcpp_action::create_server<GoToPoseAction>
        (
            this,
            "go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER READY");

        // 4. subscribe to /odom
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_sub_;
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>
        (
            "/odom", 
            10,
            std::bind(&GoToPose::odomCallback, this, std::placeholders::_1),
            sub_options
        );

        // 5. publish to the /cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>
                (
                "/cmd_vel", 
                5
                );
        timer_ = this->create_wall_timer
                (
                std::chrono::milliseconds(100), 
                std::bind(&GoToPose::timer_callback, this),
                cb_group_pub_
                );
    }

private:
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::CallbackGroup::SharedPtr cb_group_sub_;
    rclcpp::CallbackGroup::SharedPtr cb_group_pub_;

    // 3. store positions in Pose2D objects
    geometry_msgs::msg::Pose2D desired_pos_;
    geometry_msgs::msg::Pose2D current_pos_;
    geometry_msgs::msg::Pose2D delta_pos_;

    float front_theta = 0.0;
    float angular_z_vel = 0.0;
    float linear_x_vel = 0.0;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const GoToPoseAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER CALLED: GOAL RECEIVED");
        (void)uuid;
        this->desired_pos_.x = goal->goal_pos.x;
        this->desired_pos_.y = goal->goal_pos.y;
        this->desired_pos_.theta = degrees_to_radians(goal->goal_pos.theta);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER GOAL CANCELLED");
        (void)goal_handle;
        this->angular_z_vel = 0.0;
        this->linear_x_vel = 0.0;
        return rclcpp_action::CancelResponse::ACCEPT; 
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
    }

    // 6. create a control loop
    void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER RUNNING GOAL");

        auto feedback = std::make_shared<GoToPoseAction::Feedback>();
        auto result = std::make_shared<GoToPoseAction::Result>();

        rclcpp::Rate loop_rate(10);

        this->delta_pos_.x = this->desired_pos_.x - this->current_pos_.x;
        this->delta_pos_.y = this->desired_pos_.y - this->current_pos_.y;
        this->front_theta = atan2(this->delta_pos_.y, this->delta_pos_.x);
        perform_rotation(goal_handle, loop_rate, this->front_theta, feedback, result);
        if (goal_handle->is_canceling()) return;

        perform_navigation(goal_handle, loop_rate, feedback, result);
        if (goal_handle->is_canceling()) return;

        perform_rotation(goal_handle, loop_rate, this->desired_pos_.theta, feedback, result);
        if (goal_handle->is_canceling()) return;

        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER GOAL COMPLETED");
        return;
    }

    // 4. store the current odometry into current_pos_
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->current_pos_.x = msg->pose.pose.position.x;
        this->current_pos_.y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        this->current_pos_.theta = yaw;
    }

    // 5. publish to the /cmd_vel topic
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = this->linear_x_vel;
        msg.angular.z = this->angular_z_vel;
        this->publisher_->publish(msg);
    }

    // 6. create a control loop
    void perform_rotation(const std::shared_ptr<GoalHandleMove>& goal_handle, 
                        rclcpp::Rate& loop_rate, 
                        float target_theta,
                        const std::shared_ptr<GoToPoseAction::Feedback>& feedback,
                        const std::shared_ptr<GoToPoseAction::Result>& result)
    {
        this->delta_pos_.theta = normalize_angle(target_theta - this->current_pos_.theta);
        float angle_error = fabs(this->delta_pos_.theta);
        RCLCPP_INFO(this->get_logger(), "Starting Turn");
        while(angle_error > 0.025)
        {
            if (goal_handle->is_canceling())
            {
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "ACTION SERVER CANCELLING GOAL");
                return;
            }

            if(this->delta_pos_.theta > 0.0)
            {
                this->angular_z_vel = 0.25;
            }
            else
            {
                this->angular_z_vel = -0.25;
            }
            this->delta_pos_.theta = normalize_angle(target_theta - this->current_pos_.theta);
            angle_error = fabs(this->delta_pos_.theta);

            feedback->current_pos = this->current_pos_;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
        this->angular_z_vel = 0.0;
        this->linear_x_vel = 0.0;
        RCLCPP_INFO(this->get_logger(), "Turn Complete");
        loop_rate.sleep();
    }

    // 6. create a control loop
    void perform_navigation(const std::shared_ptr<GoalHandleMove>& goal_handle, 
                        rclcpp::Rate& loop_rate,
                        const std::shared_ptr<GoToPoseAction::Feedback>& feedback,
                        const std::shared_ptr<GoToPoseAction::Result>& result)
    {
        float distance_error = 1.0;
        RCLCPP_INFO(this->get_logger(), "Navigation");
        while(distance_error > 0.4)
        {
            if (goal_handle->is_canceling())
            {
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "ACTION SERVER CANCELLING GOAL");
                return;
            }

            // define a vector between desired vs current pos
            this->delta_pos_.x = this->desired_pos_.x - this->current_pos_.x;
            this->delta_pos_.y = this->desired_pos_.y - this->current_pos_.y;

            this->linear_x_vel = 0.1; // fixed linear speed of 0.2
            this->front_theta = atan2(this->delta_pos_.y, this->delta_pos_.x);
            this->angular_z_vel = this->front_theta / 2; // From section 1: angular_z_vel_ = direction_ / 2
            
            feedback->current_pos = this->current_pos_;
            goal_handle->publish_feedback(feedback);

            distance_error = sqrt(this->delta_pos_.x * this->delta_pos_.x + this->delta_pos_.y * this->delta_pos_.y);
            loop_rate.sleep();
        }
        this->angular_z_vel = 0.0;
        this->linear_x_vel = 0.0;
        RCLCPP_INFO(this->get_logger(), "Navigation Complete");
        loop_rate.sleep();
    }

    // helper methods for readable code
    float normalize_angle(float angle) 
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    double degrees_to_radians(double theta_degrees) {
        return theta_degrees * (M_PI / 180.0);
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
