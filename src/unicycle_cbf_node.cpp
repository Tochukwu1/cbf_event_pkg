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
    UnicycleCBFNode() : Node("unicycle_cbf_node"), odom_received_(false)
    {
        declare_all_parameters();
        load_parameters();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&UnicycleCBFNode::state_callback, this, _1));

        auto timer_period = std::chrono::milliseconds(timer_period_ms_);
        trigger_timer_ = this->create_wall_timer(
            timer_period, std::bind(&UnicycleCBFNode::trigger_loop, this));

        q_current_ = Eigen::Vector3d::Zero();
        u_held_ = Eigen::Vector2d::Zero();    

        RCLCPP_INFO(this->get_logger(), "Parameterized ETC-CBF Node Initialized. Awaiting Odometry...");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr trigger_timer_;

    bool odom_received_;
    Eigen::Vector3d q_current_; 
    Eigen::Vector2d u_held_;   
    
    Eigen::Vector2d obs_center_;
    Eigen::Vector2d goal_;
    
    double robot_radius_;
    double obs_radius_;
    double epsilon_;         
    double lookahead_dist_;  
    double k_p_;             
    double max_v_;
    double max_w_;
    double goal_tolerance_;
    int timer_period_ms_;

    const double MATH_TOL = 1e-6;
    const double START_TOL = 0.01;

    void declare_all_parameters()
    {
        this->declare_parameter("robot_radius", 0.18);  // Approximate radius of the TurtleBot
        this->declare_parameter("obs_x", 1.5);
        this->declare_parameter("obs_y", -0.1);
        this->declare_parameter("obs_radius", 0.5);
        
        this->declare_parameter("goal_x", 3.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("goal_tolerance", 0.05);

        this->declare_parameter("epsilon", 0.01);
        this->declare_parameter("lookahead_dist", 0.15);
        this->declare_parameter("k_p", 0.5);

        this->declare_parameter("max_v", 0.22);
        this->declare_parameter("max_w", 2.84);
        this->declare_parameter("timer_period_ms", 10);
    }

    void load_parameters()
    {

        robot_radius_ = this->get_parameter("robot_radius").as_double();    


        obs_center_ = Eigen::Vector2d(
            this->get_parameter("obs_x").as_double(),
            this->get_parameter("obs_y").as_double()
        );
        obs_radius_ = this->get_parameter("obs_radius").as_double();

        goal_ = Eigen::Vector2d(
            this->get_parameter("goal_x").as_double(),
            this->get_parameter("goal_y").as_double()
        );
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        epsilon_ = this->get_parameter("epsilon").as_double();
        lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();
        k_p_ = this->get_parameter("k_p").as_double();

        max_v_ = this->get_parameter("max_v").as_double();
        max_w_ = this->get_parameter("max_w").as_double();
        timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();

        RCLCPP_INFO(this->get_logger(), 
            "--- Parameters Loaded ---\n"
            "Goal: (%.2f, %.2f) | Obstacle: (%.2f, %.2f, r=%.2f)\n"
            "Gains: k_p=%.2f, eps=%.2f, lookahead=%.2fm",
            goal_(0), goal_(1), obs_center_(0), obs_center_(1), obs_radius_, k_p_, epsilon_, lookahead_dist_);
    }

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
        if (!odom_received_ || lookahead_dist_ < MATH_TOL) {
            return;
        }

        double dist_to_goal = (q_current_.head<2>() - goal_).norm();

        if (dist_to_goal < goal_tolerance_) {
            u_held_ = Eigen::Vector2d::Zero();
            publish_control(u_held_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached! Idling.");
            return;
        }

        // 1. Calculate Nominal Control
        Eigen::Vector2d p = get_lookahead_point(q_current_);
        Eigen::Vector2d p_dot_des = -k_p_ * (p - goal_); 
        
        double theta = q_current_(2);
        Eigen::Matrix2d A_inv;
        A_inv << cos(theta), sin(theta),
                -sin(theta) / lookahead_dist_, cos(theta) / lookahead_dist_;
                
        Eigen::Vector2d u_nom = A_inv * p_dot_des;

        // 2. Evaluate EXACT ETC Violation Condition
        static double prev_h = 0.0;
        static bool first_run = true;

        double h = h_barrier(q_current_);
        if (!first_run)
        {
            double numerical_hdot = (h - prev_h) / (timer_period_ms_ / 1000.0);

            double analytical_hdot = Lf_h(q_current_) + Lg_h(q_current_).dot(u_held_);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "h: %.3f | h_dot_num: %.3f | h_dot_ana: %.3f | diff: %.3f",
                h, numerical_hdot, analytical_hdot, numerical_hdot - analytical_hdot);
        }

        prev_h = h;
        first_run = false;


        double Lfh_val = Lf_h(q_current_);
        Eigen::RowVector2d Lgh_val = Lg_h(q_current_);

        double b = -alpha_fun(h) - Lfh_val;
        double diff = b - Lgh_val.dot(u_held_); 

        // 3. Trigger Logic
        bool is_cbf_trigger = (diff >= epsilon_);
        bool is_startup = (u_held_.norm() < 0.01);
        
        // Recovery logic: Only recover if we are safe AND our current held control is wildly different from the nominal path.
        double nominal_diff = b - Lgh_val.dot(u_nom);
        bool is_recovery = false;
        
        if (!is_cbf_trigger && (nominal_diff < -0.05)) {
             // If the held control is drifting more than 0.1 rad/s or m/s from the nominal, trigger a reset.
             if ((u_held_ - u_nom).norm() > 0.1) {
                 is_recovery = true;
             }
        }

        // 4. Update ZOH Control if Triggered
        if (is_cbf_trigger || is_recovery || is_startup) {
            
            Eigen::Vector2d u_safe = solve_cbf_qp(q_current_, u_nom, b, Lgh_val);
            if (std::abs(u_safe(0)) > max_v_ || std::abs(u_safe(1)) > max_w_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "CBF-QP solution exceeds limits! v: %.2f, w: %.2f", u_safe(0), u_safe(1));
            }
            
            if (is_cbf_trigger) {
                // THE ETC KICK
                u_held_ = u_safe + (1.0 / epsilon_) * Lgh_val.transpose();
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "[SAFETY TRIGGER] CBF update. diff: %.3f",
                    diff);
            } else {
                // Safe Update (Startup or Recovery)
                u_held_ = u_safe;
                if (is_recovery) {
                    RCLCPP_INFO(this->get_logger(), "[RECOVERY TRIGGER] Cleared obstacle, returning to nominal.");
                }
            }

            u_held_(0) = std::clamp(u_held_(0), -max_v_, max_v_);
            u_held_(1) = std::clamp(u_held_(1), -max_w_, max_w_);

            double residual = Lgh_val.dot(u_held_) - b;
            if (residual < -1e-6)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "Published control violates CBF! residual=%f",
                    residual);
            }
        }

        //  COLLISION DETECTION LOGIC 
        double physical_dist = (q_current_.head<2>() - obs_center_).norm();
        double collision_radius = robot_radius_ + obs_radius_;

        if (physical_dist <= collision_radius) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "! PHYSICAL COLLISION DETECTED! Distance to obstacle: %.2fm | h: %.2f", 
                physical_dist, h);
        } else if (h <= 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "! MATH BARRIER BREACHED! h = %.3f", h);
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[HEARTBEAT] Pos: (%.2f, %.2f) | Goal Dist: %.2f | Barrier h: %.2f",
            q_current_(0), q_current_(1), dist_to_goal, h);

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
        // use the lookahead point for the barrier function to ensure relative degree 1
        Eigen::Vector2d p = get_lookahead_point(q);

        double safe_radius =
            robot_radius_ + obs_radius_ + 0.05;

        return (p - obs_center_).squaredNorm() - (safe_radius * safe_radius);
    }

    double Lf_h(const Eigen::Vector3d& /*q*/) { return 0.0; }

    Eigen::RowVector2d Lg_h(const Eigen::Vector3d& q)
    {
        double theta = q(2);
        Eigen::Vector2d p = get_lookahead_point(q); // Must use lookahead

        Eigen::Vector2d diff = p - obs_center_;
        
        // The Jacobian mapping [v, w] -> [\dot{p_x}, \dot{p_y}]
        Eigen::Matrix2d G;
        G <<
            std::cos(theta), -lookahead_dist_ * std::sin(theta),
            std::sin(theta),  lookahead_dist_ * std::cos(theta);
             
        return 2.0 * diff.transpose() * G; 
    }

    double alpha_fun(double h) { return 1.0 * h; }

    Eigen::Vector2d solve_cbf_qp(const Eigen::Vector3d& q, const Eigen::Vector2d& u_nom, double b_cbf, const Eigen::RowVector2d& Lgh_val)
    {
        if (Lgh_val.dot(u_nom) >= b_cbf) {
            return u_nom;
        }

        double A_norm_sq = Lgh_val.squaredNorm();
        
        if (A_norm_sq < MATH_TOL) {
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