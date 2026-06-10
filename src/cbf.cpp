#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class UnicycleCBFNode : public rclcpp::Node
{
public:
    UnicycleCBFNode() : Node("unicycle_cbf_node"), odom_received_(false), epsilon_(0.1), lookahead_dist_(0.15)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&UnicycleCBFNode::state_callback, this, _1));

        // 20Hz loop (Gazebo expects continuous publishing)
        trigger_timer_ = this->create_wall_timer(
            50ms, std::bind(&UnicycleCBFNode::trigger_loop, this));

        q_current_ = Eigen::Vector3d::Zero();
        u_held_ = Eigen::Vector2d::Zero();    
        
        obs_center_ = Eigen::Vector2d(1.5, -0.1);
        obs_radius_ = 0.5;

        RCLCPP_INFO(this->get_logger(), "Unicycle CBF Node Initialized. Awaiting Odometry...");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr trigger_timer_;

    bool odom_received_;
    Eigen::Vector3d q_current_; 
    Eigen::Vector2d u_held_;   
    Eigen::Vector2d obs_center_;
    
    double obs_radius_;
    double epsilon_;         // Event-trigger safety margin
    double lookahead_dist_;     

    void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        q_current_(0) = msg->pose.pose.position.x;
        q_current_(1) = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        q_current_(2) = yaw;
        odom_received_ = true;
    }

    void trigger_loop()
    {
        // 0. Wait until we have real data from Gazebo
        if (!odom_received_) {
            return;
        }

        // Guard against divide-by-zero if lookahead is accidentally set to 0
        if (lookahead_dist_ < 1e-6) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                  "Lookahead distance too small! Controller halted.");
            return;
        }

        // Stop condition: if the robot's physical center reaches the origin, halt.
        if (q_current_.head<2>().norm() < 0.05) {
            u_held_ = Eigen::Vector2d::Zero();
            publish_control(u_held_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached! Idling.");
            return;
        }

        // 1. Nominal control: PD Controller for Regulation to Origin using lookahead
        Eigen::Vector2d p = get_lookahead_point(q_current_);
        double k_p = 0.5; // Proportional gain
        Eigen::Vector2d p_dot_des = -k_p * p; 
        
        double theta = q_current_(2);
        Eigen::Matrix2d A_inv;
        A_inv << cos(theta), sin(theta),
                -sin(theta) / lookahead_dist_, cos(theta) / lookahead_dist_;
                
        Eigen::Vector2d u_nom = A_inv * p_dot_des;

        // 2. Trigger if the currently held control violates the safety margin (eps), or if starting
        if (check_trigger_condition(q_current_, u_held_, epsilon_) || u_held_.norm() < 0.01) {
            
            // Solve the actual CBF-QP to guarantee safety
            u_held_ = solve_cbf_qp(q_current_, u_nom);

            // Enforce TurtleBot hardware limits
            double max_v = 0.22;  
            double max_w = 2.84;  
            u_held_(0) = std::clamp(u_held_(0), -max_v, max_v);
            u_held_(1) = std::clamp(u_held_(1), -max_w, max_w);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Triggered! Updated control -> v: %.2f, w: %.2f", u_held_(0), u_held_(1));
        }

        // 3. CONTINUOUSLY publish the held control
        publish_control(u_held_);
    }

    void publish_control(const Eigen::Vector2d& u)
    {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = u(0);  
        cmd.angular.z = u(1); 
        cmd_pub_->publish(cmd);
    }

    Eigen::Vector2d get_lookahead_point(const Eigen::Vector3d& q)
    {
        return Eigen::Vector2d(q(0) + lookahead_dist_ * cos(q(2)), 
                               q(1) + lookahead_dist_ * sin(q(2)));
    }

    double h_barrier(const Eigen::Vector3d& q)
    {
        Eigen::Vector2d p = get_lookahead_point(q);
        // Positive outside the obstacle, negative inside
        return (p - obs_center_).squaredNorm() - (obs_radius_ * obs_radius_);
    }

    double Lf_h(const Eigen::Vector3d& /*q*/) { return 0.0; }

    Eigen::RowVector2d Lg_h(const Eigen::Vector3d& q)
    {
        double theta = q(2);
        Eigen::Vector2d p = get_lookahead_point(q);
        
        Eigen::Matrix2d A;
        A << cos(theta), -lookahead_dist_ * sin(theta),
             sin(theta),  lookahead_dist_ * cos(theta);
             
        Eigen::Vector2d diff = p - obs_center_;
        // Use RowVector2d for correct dot product dimensions downstream
        return 2.0 * diff.transpose() * A; 
    }

    double alpha_fun(double h) { return 1.0 * h; }

    bool check_trigger_condition(const Eigen::Vector3d& q, const Eigen::Vector2d& u, double eps)
    {
        double h = h_barrier(q);
        double Lfh_val = Lf_h(q);
        Eigen::RowVector2d Lgh_val = Lg_h(q);
        
        // Standard CBF Condition: Lf_h + Lg_h*u + alpha(h) >= 0
        double cbf_value = Lfh_val + Lgh_val.dot(u) + alpha_fun(h);
        
        // Trigger an update if the safety value drops below our chosen margin (eps)
        return cbf_value < eps; 
    }

    Eigen::Vector2d solve_cbf_qp(const Eigen::Vector3d& q, const Eigen::Vector2d& u_nom)
    {
        double h = h_barrier(q);
        double Lfh_val = Lf_h(q);
        Eigen::RowVector2d Lgh_val = Lg_h(q);

        double b_cbf = -Lfh_val - alpha_fun(h);

        // If the nominal control is already safe, use it directly
        if (Lgh_val.dot(u_nom) >= b_cbf) {
            return u_nom;
        }

        // Otherwise, solve the closed-form KKT conditions for 1 constraint:
        // min ||u - u_nom||^2  s.t.  A*u >= b
        // Solution: u* = u_nom + A^T * (b - A*u_nom) / (A*A^T)
        double A_norm_sq = Lgh_val.squaredNorm();
        
        // Prevent division by zero if gradient vanishes (e.g. exactly at obstacle center)
        if (A_norm_sq < 1e-6) {
            return Eigen::Vector2d::Zero();
        }

        double lambda = (b_cbf - Lgh_val.dot(u_nom)) / A_norm_sq;
        
        return u_nom + lambda * Lgh_val.transpose();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnicycleCBFNode>());
    rclcpp::shutdown();
    return 0;
}