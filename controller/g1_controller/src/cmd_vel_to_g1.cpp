#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <array>
#include <vector>

class CmdVelToG1 : public rclcpp::Node
{
public:
    CmdVelToG1() : Node("cmd_vel_to_g1")
    {
        const char *home = std::getenv("HOME");
        std::string home_dir = std::string(home ? home : "");
        std::string default_g1_loco_client_path =
            home_dir + "/reynaldy_ws/unitree_g1_control/unitree_sdk2/build/bin/g1_loco_client";

        // Declare parameters
        this->declare_parameter<std::string>("network_interface", "eno1");
        this->declare_parameter<std::string>("g1_loco_client_path", default_g1_loco_client_path);
        this->declare_parameter<double>("command_timeout", 0.1); // seconds
        this->declare_parameter<bool>("send_stop_on_zero_cmd", true);
        this->declare_parameter<double>("max_x_linear_velocity", 0.5); // m/s
        this->declare_parameter<double>("max_y_linear_velocity", 0.5); // m/s
        this->declare_parameter<double>("max_angular_velocity", 0.5); // rad/s
        
        // Get parameters
        this->get_parameter("network_interface", network_interface_);
        this->get_parameter("g1_loco_client_path", g1_loco_client_path_);
        this->get_parameter("command_timeout", command_timeout_);
        this->get_parameter("send_stop_on_zero_cmd", send_stop_on_zero_cmd_);
        this->get_parameter("max_x_linear_velocity", max_x_linear_vel_);
        this->get_parameter("max_y_linear_velocity", max_y_linear_vel_);
        this->get_parameter("max_angular_velocity", max_angular_vel_);

        if (!std::filesystem::exists(g1_loco_client_path_))
        {
            std::vector<std::string> candidates;
            if (!home_dir.empty())
            {
                candidates.emplace_back(home_dir + "/reynaldy_ws/unitree_g1_control/unitree_sdk2/build/bin/g1_loco_client");
                candidates.emplace_back(home_dir + "/unitree_g1_control/unitree_sdk2/build/bin/g1_loco_client");
            }

            for (const auto &candidate : candidates)
            {
                if (std::filesystem::exists(candidate))
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Configured g1_loco_client_path does not exist (%s). Using detected path: %s",
                        g1_loco_client_path_.c_str(),
                        candidate.c_str());
                    g1_loco_client_path_ = candidate;
                    break;
                }
            }
        }
        
        // Create subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&CmdVelToG1::cmdVelCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "G1 cmd_vel bridge node started");
        RCLCPP_INFO(this->get_logger(), "Network interface: %s", network_interface_.c_str());
        RCLCPP_INFO(this->get_logger(), "G1 loco client path: %s", g1_loco_client_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Command timeout: %.2f seconds", command_timeout_);
        RCLCPP_INFO(this->get_logger(), "Send stop on zero cmd: %s", send_stop_on_zero_cmd_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Max X linear velocity: %.2f m/s", max_x_linear_vel_);
        RCLCPP_INFO(this->get_logger(), "Max Y linear velocity: %.2f m/s", max_y_linear_vel_);
        RCLCPP_INFO(this->get_logger(), "Max angular velocity: %.2f rad/s", max_angular_vel_);
        RCLCPP_INFO(this->get_logger(), "Listening to /cmd_vel...");

        logClientProbe();
    }

private:
    double clampSymmetric(double value, double max_abs)
    {
        if (value > max_abs) return max_abs;
        if (value < -max_abs) return -max_abs;
        return value;
    }

    double clampForwardOnly(double value, double max_value)
    {
        if (value > max_value) return max_value;
        if (value < 0.0) return 0.0;
        return value;
    }

    bool isZeroCommand(double linear_x, double linear_y, double angular_z) const
    {
        constexpr double eps = 1e-6;
        return std::abs(linear_x) < eps &&
               std::abs(linear_y) < eps &&
               std::abs(angular_z) < eps;
    }

    std::pair<int, std::string> runCommand(const std::string &command)
    {
        std::array<char, 256> buffer{};
        std::string output;

        FILE *pipe = popen((command + " 2>&1").c_str(), "r");
        if (pipe == nullptr)
        {
            return {-1, "Failed to start subprocess with popen()."};
        }

        while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr)
        {
            output += buffer.data();
        }

        int status = pclose(pipe);
        return {status, output};
    }

    void logClientProbe()
    {
        std::ostringstream probe;
        probe << g1_loco_client_path_
              << " --network_interface=" << network_interface_
              << " --get_fsm_id";

        const auto [status, output] = runCommand(probe.str());
        if (status == 0)
        {
            RCLCPP_INFO(this->get_logger(), "SDK probe succeeded:\n%s", output.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "SDK probe failed with code %d:\n%s", status, output.c_str());
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto current_time = this->now();
        
        // Check if enough time has passed since last command (safety debouncing)
        if ((current_time - last_command_time_).seconds() < command_timeout_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Command rate too high, skipping command for safety");
            return;
        }
        
        // Clamp velocities for safety and command-interface constraints:
        // x: forward only, y/yaw: allow positive and negative.
        double linear_x = clampForwardOnly(msg->linear.x, max_x_linear_vel_);
        double linear_y = clampSymmetric(msg->linear.y, max_y_linear_vel_);
        double angular_z = clampSymmetric(msg->angular.z, max_angular_vel_);
        
        // Log if velocities were clamped
        if (msg->linear.x < 0.0 || msg->linear.x > max_x_linear_vel_ ||
            msg->linear.y < -max_y_linear_vel_ || msg->linear.y > max_y_linear_vel_ ||
            msg->angular.z < -max_angular_vel_ || msg->angular.z > max_angular_vel_)
        {
            RCLCPP_WARN(this->get_logger(), 
                "Velocity clamped for safety! Requested: [%.2f, %.2f, %.2f] -> Clamped: [%.2f, %.2f, %.2f]",
                msg->linear.x, msg->linear.y, msg->angular.z,
                linear_x, linear_y, angular_z);
        }

        std::ostringstream command;
        command << g1_loco_client_path_
                << " --network_interface=" << network_interface_;

        if (send_stop_on_zero_cmd_ && isZeroCommand(linear_x, linear_y, angular_z))
        {
            command << " --stop_move";
        }
        else
        {
            std::ostringstream move_values;
            move_values << linear_x << " " << linear_y << " " << angular_z;
            command << " --move=\"" << move_values.str() << "\"";
        }
        
        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.str().c_str());
        
        const auto [result, output] = runCommand(command.str());
        
        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Command executed successfully: linear_x=%.2f, linear_y=%.2f, angular_z=%.2f",
                        linear_x, linear_y, angular_z);
            if (!output.empty())
            {
                RCLCPP_INFO(this->get_logger(), "g1_loco_client output:\n%s", output.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Command execution failed with code: %d\n%s", result, output.c_str());
        }
        
        // Update last command time
        last_command_time_ = current_time;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::string network_interface_;
    std::string g1_loco_client_path_;
    double command_timeout_;
    bool send_stop_on_zero_cmd_;
    double max_x_linear_vel_;
    double max_y_linear_vel_;
    double max_angular_vel_;
    rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try
    {
        auto node = std::make_shared<CmdVelToG1>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("cmd_vel_to_g1"), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
