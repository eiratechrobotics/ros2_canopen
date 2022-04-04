
#ifndef MC_DEVICE_NODE_HPP
#define MC_DEVICE_NODE_HPP

#include "motion_controller_driver/visibility_control.h"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ros2_canopen_interfaces/srv/co_target_double.hpp"
#include "proxy_device_driver/proxy_device_driver.hpp"
#include "motion_controller_driver/motor.hpp"

using namespace std::chrono_literals;
using namespace ros2_canopen;
using namespace canopen_402;
namespace ros2_canopen
{
    /**
     * @brief ROS2 node for a Motion Controller
     *
     * This class provides a ros2 node for a CIA 402
     * device.
     */
    class MotionControllerDriver : public ProxyDeviceDriver
    {
    private:
        std::shared_ptr<MCDeviceDriver> mc_driver_;
        std::shared_ptr<Motor402> motor_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_init_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_halt_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_recover_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_position_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_torque_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_velocity_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_cyclic_velocity_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_cyclic_position_service;
        rclcpp::Service<ros2_canopen_interfaces::srv::COTargetDouble>::SharedPtr handle_set_target_service;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_subscription;
        rclcpp::CallbackGroup::SharedPtr timer_group;
        bool intialised;
        void register_services();

    public:
        explicit MotionControllerDriver(const rclcpp::NodeOptions &options)
            : ProxyDeviceDriver(options)
        {
            intialised = false;
        }

        void run()
        {
            if(!intialised)
            {
                RCLCPP_INFO(this->get_logger(), "Intitialise");
                timer_->cancel();
                intialised = true;
                motor_->registerDefaultModes();
                mc_driver_->validate_objs();
                timer_= this->create_wall_timer(
                        100ms, std::bind(&MotionControllerDriver::run, this), timer_group);
            }
            
            motor_->handleRead();
            motor_->handleWrite();
            motor_->handleDiag();
        }

        void init(ev::Executor &exec,
                  canopen::AsyncMaster &master,
                  uint8_t node_id) noexcept override;

    protected:
        virtual void on_rpdo(COData data) override
        {
            RCLCPP_INFO(this->get_logger(), "on_rpo not implemented");
        }

    private:
        std::atomic<bool> active;

        /**
         * @brief Service Callback to initialise device
         * 
         * Calls Motor402::handleInit function. Brings motor to enabled
         * state and homes it.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_init(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to recover device
         * 
         * Calls Motor402::handleRecover function. Resets faults and brings
         * motor to enabled state.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_recover(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to halt device
         * 
         * Calls Motor402::handleHalt function. Calls Quickstop. Resulting
         * Motor state depends on devices configuration specifically object
         * 0x605A.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_halt(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set profiled position mode
         * 
         * Calls Motor402::enterModeAndWait with Profiled Position Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Profiled Position Mode.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_set_mode_position(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set profiled velocity mode
         * 
         * Calls Motor402::enterModeAndWait with Profiled Velocity Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Profiled Velocity Mode.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_set_mode_velocity(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set cyclic position mode
         * 
         * Calls Motor402::enterModeAndWait with Cyclic Position Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Cyclic Position Mode.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_set_mode_cyclic_position(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);


        /**
         * @brief Service Callback to set cyclic velocity mode
         * 
         * Calls Motor402::enterModeAndWait with Cyclic Velocity Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Cyclic Velocity Mode.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_set_mode_cyclic_velocity(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set profiled torque mode
         * 
         * Calls Motor402::enterModeAndWait with Profiled Torque Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Profiled Torque Mode.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_set_mode_torque(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set target
         * 
         * Calls Motor402::setTarget and sets the requested target value. Note
         * that the resulting movement is dependent on the Operation Mode and the
         * drives state.
         * 
         * @param [in] request 
         * @param [out] response 
         */
        void handle_set_target(
            const ros2_canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
            ros2_canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response);

        /**
         * @brief Subscription Callback to set target
         *
         * Calls Motor402::setTarget and sets the requested target value. Note
         * that the resulting movement is dependent on the Operation Mode and the
         * drives state.
         * 
         * @param [in] msg 
         */
        void target_callback(const std_msgs::msg::Float64 msg) const;
    };

}

#endif