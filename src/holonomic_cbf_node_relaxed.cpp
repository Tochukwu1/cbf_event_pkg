#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>
#include <numbers>
#include <fstream> // for CSV logging
#include <filesystem> // for checking paths
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/Dense>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class HolonomicCBFNode : public rclcpp::Node
{
public:
    HolonomicCBFNode() : Node("holonomic_cbf_node"), state_received_(false)
    {
        declare_all_parameters();
        load_parameters();
        
        // --- CSV Initialization ---
        std::string file_path = this->get_parameter("log_file_path").as_string();
        
        csv_file_.open(file_path);
        if (csv_file_.is_open()) {
            csv_file_ << "time,x,y,theta,u_x,u_y,mode\n";
            RCLCPP_INFO(this->get_logger(), "Logging data to: %s", file_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV for logging at: %s", file_path.c_str());
        }
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        
        state_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/holonomic_robot/pose", 1,
                            std::bind(&HolonomicCBFNode::state_callback, this, _1));

        auto timer_period = std::chrono::milliseconds(timer_period_ms_);
        trigger_timer_ = this->create_wall_timer(timer_period, std::bind(&HolonomicCBFNode::trigger_loop, this));

        q_current_ = Eigen::Vector3d::Zero(); 
        u_held_ = Eigen::Vector2d::Zero();    

        RCLCPP_INFO(this->get_logger(), "Holonomic ETC-CBF Node Initialized. Awaiting TF pose...");
    }

    // --- Destructor to close CSV properly ---
    ~HolonomicCBFNode() {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "Saved and closed simulation data log.");
        }
    }

private:
    enum class ControlMode { INIT, NOMINAL, CBF_ACTIVE, GOAL_REACHED };
    ControlMode current_mode_ = ControlMode::INIT;

    std::ofstream csv_file_; // File stream for logging

    std::string mode_to_string(ControlMode mode) {
        switch (mode) {
            case ControlMode::INIT: return "INIT";
            case ControlMode::NOMINAL: return "NOMINAL";
            case ControlMode::CBF_ACTIVE: return "CBF_ACTIVE";
            case ControlMode::GOAL_REACHED: return "GOAL_REACHED";
            default: return "UNKNOWN";
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr trigger_timer_;

    bool state_received_;
    Eigen::Vector3d q_current_;
    Eigen::Vector2d u_held_;

    Eigen::Vector2d obs_center_;
    Eigen::Vector2d goal_;

    double robot_radius_;
    double obs_radius_;
    double epsilon_; 
    double k_p_; 
    double max_v_; 
    double goal_tolerance_; 
    int timer_period_ms_; 

    const double MATH_TOL = 1e-6;

    void declare_all_parameters()
    {
        // Added parameter for the CSV log file path
        this->declare_parameter<std::string>("log_file_path", "sim_data.csv");
        
        this->declare_parameter("robot_radius", 0.18);
        this->declare_parameter("obs_x", 1.5);
        this->declare_parameter("obs_y", -0.1);
        this->declare_parameter("obs_radius", 0.5);

        this->declare_parameter("goal_x", 3.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("goal_tolerance", 0.05);

        this->declare_parameter("epsilon", 0.01);
        this->declare_parameter("k_p", 0.5);

        this->declare_parameter("max_v", 1.0);
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
        k_p_ = this->get_parameter("k_p").as_double();
        max_v_ = this->get_parameter("max_v").as_double();
        timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();
    }

    void state_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform_stamped : msg->transforms)
        {
            if (transform_stamped.child_frame_id == "holonomic_robot")
            {
                q_current_(0) = transform_stamped.transform.translation.x;
                q_current_(1) = transform_stamped.transform.translation.y;

                tf2::Quaternion q(
                    transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z,
                    transform_stamped.transform.rotation.w
                );

                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                yaw = std::remainder(yaw, 2.0 * M_PI);

                q_current_(2) = yaw;
                state_received_ = true; 

                break; 
            }
        }
    }

    void trigger_loop()
    {
        if (!state_received_) return;

        Eigen::Vector2d p = q_current_.head<2>();
        double dist_to_goal = (p - goal_).norm();

        if (dist_to_goal < goal_tolerance_)
        {
            u_held_ = Eigen::Vector2d::Zero();
            publish_control(u_held_);
            
            if (current_mode_ != ControlMode::GOAL_REACHED) {
                RCLCPP_INFO(this->get_logger(), "=== GOAL REACHED! Idling. ===");
                current_mode_ = ControlMode::GOAL_REACHED;
            }
            log_to_csv(); // Log final resting states
            return;
        }

        Eigen::Vector2d u_nom = -k_p_ * (p - goal_);

        double h = h_barrier(q_current_);
        double Lfh_val = Lf_h(q_current_);
        Eigen::RowVector2d Lgh_val = Lg_h(q_current_);

        double b = -alpha_fun(h) - Lfh_val;
        double diff = b - Lgh_val.dot(u_held_);

        double update_thresh = std::min(0.05, 0.2 * dist_to_goal);
        double nom_error = (u_held_ - u_nom).norm();

        bool is_cbf_trigger = (diff >= epsilon_);
        bool is_startup = (u_held_.norm() < 0.01 && current_mode_ == ControlMode::INIT);
        bool is_nominal_update = (nom_error > update_thresh);

        // --- RESTORED TELEMETRY SNAPSHOT ---
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "\n--- TELEMETRY SNAPSHOT ---\n"
            " Mode      : %s\n"
            " Dist2Goal : %.3f m\n"
            " h (Safety): %.3f (Collision if < 0)\n"
            " CBF Diff  : %.3f (Triggers CBF if > %.3f)\n"
            " Nom Error : %.3f (Triggers Update if > %.3f)\n"
            " u_held    : [%.3f, %.3f]",
            mode_to_string(current_mode_).c_str(), dist_to_goal, h, 
            diff, epsilon_, nom_error, update_thresh, u_held_(0), u_held_(1));

        if (is_cbf_trigger || is_nominal_update || is_startup)
        {
            Eigen::Vector2d u_safe = solve_cbf_qp(q_current_, u_nom, b, Lgh_val);
            u_held_ = u_safe; 

            bool is_cbf_intervening = (Lgh_val.dot(u_nom) < b);

            // --- RESTORED MODE TRANSITION LOGS ---
            if (is_cbf_intervening)
            {
                if (current_mode_ != ControlMode::CBF_ACTIVE) {
                    RCLCPP_WARN(this->get_logger(), ">> SAFETY BOUND TRIGGERED! Actively avoiding obstacle.");
                    current_mode_ = ControlMode::CBF_ACTIVE;
                }
            }
            else
            {
                if (current_mode_ == ControlMode::CBF_ACTIVE) {
                    RCLCPP_INFO(this->get_logger(), ">> OBSTACLE CLEARED! Recovering to nominal trajectory.");
                    current_mode_ = ControlMode::NOMINAL;
                } else if (is_startup) {
                    RCLCPP_INFO(this->get_logger(), ">> SYSTEM STARTUP. Moving on nominal trajectory.");
                    current_mode_ = ControlMode::NOMINAL;
                }
            }

            double speed = u_held_.norm();
            if (speed > max_v_) {
                u_held_ = (u_held_ / speed) * max_v_;
            }
        }

        publish_control(u_held_);
        log_to_csv(); // Log current loop execution
    }

    void publish_control(const Eigen::Vector2d& u_global)
    {
        double theta = q_current_(2); 

        double vx_local = u_global(0) * std::cos(theta) + u_global(1) * std::sin(theta);
        double vy_local = -u_global(0) * std::sin(theta) + u_global(1) * std::cos(theta);

        vx_local = std::clamp(vx_local, -max_v_, max_v_);
        vy_local = std::clamp(vy_local, -max_v_, max_v_);

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = vx_local; 
        cmd.linear.y = vy_local; 
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    void log_to_csv()
    {
        if (csv_file_.is_open()) {
            csv_file_ << this->now().seconds() << ","
                      << q_current_(0) << ","
                      << q_current_(1) << ","
                      << q_current_(2) << ","
                      << u_held_(0) << ","
                      << u_held_(1) << ","
                      << mode_to_string(current_mode_) << "\n";
        }
    }

    double h_barrier(const Eigen::Vector3d& q)
    {
        Eigen::Vector2d p = q.head<2>(); 
        double safe_radius = robot_radius_ + obs_radius_ + 0.05;
        return (p - obs_center_).squaredNorm() - (safe_radius * safe_radius);
    }

    double Lf_h(const Eigen::Vector3d& q)
    {
        (void)q; return 0.0;
    }

    Eigen::RowVector2d Lg_h(const Eigen::Vector3d& q)
    {
        Eigen::Vector2d p = q.head<2>();
        Eigen::Vector2d diff = p - obs_center_;
        return 2.0 * diff.transpose(); 
    }

    double alpha_fun(double h)
    {
        return 1.0 * h; 
    }

    Eigen::Vector2d solve_cbf_qp(const Eigen::Vector3d& q, const Eigen::Vector2d& u_nom, double b_cbf, const Eigen::RowVector2d& Lgh_val)
    {
        (void) q; 
        if (Lgh_val.dot(u_nom) >= b_cbf) { return u_nom; }

        double A_norm_sq = Lgh_val.squaredNorm();
        if (A_norm_sq < MATH_TOL) { return Eigen::Vector2d::Zero(); }

        double lambda = (b_cbf - Lgh_val.dot(u_nom)) / A_norm_sq;
        return u_nom + lambda * Lgh_val.transpose();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolonomicCBFNode>());
    rclcpp::shutdown();
    return 0;
}