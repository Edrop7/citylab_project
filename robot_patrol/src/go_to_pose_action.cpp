#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GoToPose : public rclcpp::Node
{
public:
    using GoToPose = robot_patrol::action::GoToPose;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<GoToPose>;

    explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("go_to_pose_node", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<Distance>
        (
            this,
            "go_to_pose",
            std::bind(&MyActionServer::handle_goal, this, _1, _2),
            std::bind(&MyActionServer::handle_cancel, this, _1),
            std::bind(&MyActionServer::handle_accepted, this, _1)
        );
    }
    RCLCPP_INFO(this->get_logger(), "ACTION SERVER READY");

    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>
    (
        "/odom", 
        10,
        std::bind
        (
                    &MyActionServer::odomCallback, 
                    this, 
                    std::placeholders::_1
        )
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>
            (
            "/cmd_vel", 
            5
            );

private:
    rclcpp_action::Server<Distance>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

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

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const Distance::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER GOAL RECEIVED");
        (void)uuid;
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
        std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "ACTION SERVER RUNNING GOAL");

        const auto goal = goal_handle->get_goal();
        this->goal_x = goal->goal_pos.x;
        this->goal_y = goal->goal_pos.y;
        this->goal_theta = goal->goal_pos.theta;

        auto feedback = std::make_shared<GoToPose::Feedback>();
        auto result = std::make_shared<GoToPose::Result>();

        rclcpp::Rate loop_rate(10); // 10 Hz

        while(!this->target_achieved)
        {
            if (goal_handle->is_canceling())
            {
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "ACTION SERVER CANCELLING GOAL");
                return;
            }

        }

        this->dx = this->goal_x - this->current_x;
        this->dy = this->goal_y - this->current_y;
        this->dtheta = this->goal_theta - this->current_theta;



        double distance_error = sqrt(this->dx * this->dx + this->dy * this->dy);
        double angle_error = fabs(this->dtheta);

        feedback->current_pos.x = current_x;
        feedback->current_pos.y = current_y;
        feedback->current_pos.theta = current_theta;
        goal_handle->publish_feedback(feedback);

        if (distance_error < 0.05 && angle_error < 0.1)
        {
            result->status = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "ACTION SERVER GOAL REACHED");
            return;
        }
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
} // CLASS GoToPose
