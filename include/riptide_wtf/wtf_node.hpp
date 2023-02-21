#pragma once

#include "rclcpp/rclcpp.hpp"
#include <ncurses.h>
// #include <systemd/sd-bus.h>


using namespace std::chrono_literals;
using namespace std;

namespace riptide_wtf {

    class WtfNode : public rclcpp::Node {
        public:
            WtfNode();
            ~WtfNode();

        private:
            /// Rclcpp
            rclcpp::TimerBase::SharedPtr timer_;
            std::chrono::milliseconds loop_dt_ = 1s; /// loop dt

            string hostname_ = "Riptide1";

            // sd_bus m_bus;

            /// Variable
            WINDOW *windows_robot_;
            WINDOW *windows_daemon_;
            WINDOW *windows_safety_;
            WINDOW *windows_internal_pressure_;
            WINDOW *windows_depth_;
            WINDOW *windows_power_;
            WINDOW *windows_depth_control_;
            WINDOW *windows_piston_;
            WINDOW *windows_mission_;

            /// Variable
            // seabot2_safety::msg::SafetyStatus msg_safety_;
            // seabot2_depth_filter::msg::DepthPose msg_depth_data_;
            // pressure_bme280_driver::msg::Bme280Data msg_internal_sensor_filter_;
            // seabot2_power_driver::msg::PowerState msg_power_data_;
            // seabot2_piston_driver::msg::PistonState msg_piston_data_;
            // seabot2_mission::msg::Waypoint msg_waypoint_;
            // seabot2_depth_control::msg::DepthControlDebug msg_depth_control_;

            rclcpp::Time time_last_safety_ = this->now();
            rclcpp::Time time_last_depth_data_ = this->now();
            rclcpp::Time time_last_internal_sensor_filter_ = this->now();
            rclcpp::Time time_last_power_data_ = this->now();
            rclcpp::Time time_last_piston_data_ = this->now();
            rclcpp::Time time_last_waypoint_ = this->now();
            rclcpp::Time time_last_depth_control_ = this->now();

            bool msg_first_received_safety_ = false;
            bool msg_first_received_depth_data_ = false;
            bool msg_first_received_internal_sensor_filter_ = false;
            bool msg_first_received_power_data_ = false;
            bool msg_first_received_piston_data_ = false;
            bool msg_first_received_waypoint_ = false;
            bool msg_first_received_depth_control_ = false;

            /// Interfaces
            // rclcpp::Subscription<seabot2_safety::msg::SafetyStatus>::SharedPtr subscriber_safety_;
            // rclcpp::Subscription<seabot2_depth_filter::msg::DepthPose>::SharedPtr subscriber_depth_data_;
            // rclcpp::Subscription<pressure_bme280_driver::msg::Bme280Data>::SharedPtr subscriber_internal_sensor_filter_;
            // rclcpp::Subscription<seabot2_power_driver::msg::PowerState>::SharedPtr subscriber_power_data_;
            // rclcpp::Subscription<seabot2_piston_driver::msg::PistonState>::SharedPtr subscriber_piston_data_;
            // rclcpp::Subscription<seabot2_mission::msg::Waypoint>::SharedPtr subscriber_mission_;
            // rclcpp::Subscription<seabot2_depth_control::msg::DepthControlDebug>::SharedPtr subscriber_control_debug_;

            /**
             *  Init and get parameters of the Node
             */
            void init_parameters();

            /**
             * Init interfaces of this node
             */
            void init_interfaces();

            /**
             * Timer callback
             */
            void timer_callback();

            /**
             * Depth Callback
             * @param msg
             */
            // void depth_callback(const seabot2_depth_filter::msg::DepthPose &msg);

            /**
             * Internal sensor callback
             * @param msg
             */
            // void internal_sensor_callback(const pressure_bme280_driver::msg::Bme280Data &msg);

            /**
             * Power callback
             * @param msg
             */
            // void power_callback(const seabot2_power_driver::msg::PowerState &msg);

            /**
             *  Piston callback
             * @param msg
             */
            // void piston_callback(const seabot2_piston_driver::msg::PistonState &msg);

            /**
             *  Safety callback
             * @param msg
             */
            // void safety_callback(const seabot2_safety::msg::SafetyStatus &msg);

            /**
             *  Waypoint callback
             * @param msg
             */
            // void waypoint_callback(const seabot2_mission::msg::Waypoint &msg);

            /**
             *  Control debug callback
             * @param msg
             */
            // void depth_control_callback(const seabot2_depth_control::msg::DepthControlDebug &msg);

            /**
             *  Update daemon windows
             */
            void update_daemon_windows();

            /**
             *  Update safety windows
             */
            void update_safety_windows();

            /**
             * Update mission windows
             */
            void update_mission_windows();

            /**
             *  Update internal pressure windows
             */
            void update_internal_pressure_windows();

            /**
             * Update power windows
             */
            void update_power();

            /**
             *  Update depth windows
             */
            void update_depth();

            /**
             * Update piston windows
             */
            void update_piston();

            /**
             * Update robot info windows
             */
            void update_robot();

            /**
             * Update depth windows
             */
            void update_depth_control();
    };
} // namespace riptide_wtf