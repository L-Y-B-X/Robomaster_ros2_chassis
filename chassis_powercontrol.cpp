#include"gary_chsssis/chassis_powercontrol.hpp"

#include <chrono>
#include <functional>
#include "gary_msgs/msg/detail/power_heat__struct.hpp"
#include "boost/timer/timer.hpp"
using namespace gary_chassis;
using namespace boost;


ChassisPowercontrol::ChassisPowercontrol(const rclcpp::NodeOptions &options) :
        rclcpp_lifecycle::LifecycleNode("mecanum_chassis_solver", options), a(0), b(0), r(0){
this->declare_parameter("cmd_topic", "~/cmd_vel");

}

CallbackReturn ChassisPowercontrol::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);


    if (this->get_parameter("cmd_topic").get_type() != rclcpp::PARAMETER_STRING) {
        RCLCPP_ERROR(this->get_logger(), "cmd_topic type must be string");
        return CallbackReturn::FAILURE;
    }
    this->cmd_topic = this->get_parameter("cmd_topic").as_string();
    subscription_  = this->create_subscription<gary_msgs::msg::PowerHeat>(
            this->cmd_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ChassisPowercontrol::power_callback, this, std::placeholders::_1));
using namespace std::chrono;
    this->timer_update = this->create_wall_timer(100ms , [this] {time_callback(); });
    this->timer_update = this->create_wall_timer(100ms , [this] {void power_callback(gary_msgs::msg::PowerHeat::SharedPtr msg); });
//put the time_




    //get a
    if (this->get_parameter("a").get_type() != rclcpp::PARAMETER_DOUBLE) {
        RCLCPP_ERROR(this->get_logger(), "a type must be double");
        return CallbackReturn::FAILURE;
    }
    this->a = this->get_parameter("a").as_double();

    //?  this->mecanum_kinematics = std::make_shared<gary_chassis::MecanumKinematics>(this->a, this->b, this->r);

    RCLCPP_INFO(this->get_logger(), "configured");
    return CallbackReturn::SUCCESS;
}



    CallbackReturn ChassisPowercontrol::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        this->cmd_subscriber.reset();
        this->diag_subscriber.reset();

        //this->mecanum_kinematics.reset();

        RCLCPP_INFO(this->get_logger(), "cleaning up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ChassisPowercontrol::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

   /*  subscription_ = this->create_subscription<>
      (
               subscription_, 10, std::bind(&ChassisPowercontrol::power_callback, this,std::placeholders::_1));

*/
        this->power_publisher->on_activate();


        RCLCPP_INFO(this->get_logger(), "activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ChassisPowercontrol::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        this->power_publisher->on_deactivate();


        RCLCPP_INFO(this->get_logger(), "deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ChassisPowercontrol::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->cmd_subscriber.get() != nullptr) this->cmd_subscriber.reset();
        if (this->diag_subscriber.get() != nullptr) this->diag_subscriber.reset();
        if (this->power_publisher.get() != nullptr) this->power_publisher.reset();
       // if (this->mecanum_kinematics != nullptr) this->mecanum_kinematics.reset();

        RCLCPP_INFO(this->get_logger(), "shutdown");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ChassisPowercontrol::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->cmd_subscriber.get() != nullptr) this->cmd_subscriber.reset();
        if (this->diag_subscriber.get() != nullptr) this->diag_subscriber.reset();
        if (this->power_publisher.get() != nullptr) this->power_publisher.reset();

        //if (this->mecanum_kinematics != nullptr) this->mecanum_kinematics.reset();

        RCLCPP_INFO(this->get_logger(), "error");
        return CallbackReturn::SUCCESS;
    }
gary_msgs::msg::PowerHeat power;
void power_callback(gary_msgs::msg::PowerHeat::SharedPtr msg)
{
    power = *msg;
}

void time_callback()
{

    const float warning_power_buffer = 50.0,warning_power = 40.0;
    const float buffer_total_current_limit = 16000.0 , power_total_current_limit = 20000.0;
    float total_current_limit = 0.0;
    if(power.chassis_power_buffer < warning_power_buffer)
    {
        float power_scale;
        if(power.chassis_power_buffer > 5.0)
        {
            power_scale = power.chassis_power_buffer / warning_power_buffer;
        }
        else{
            power_scale = 5.0f / warning_power_buffer;
        }
        total_current_limit = buffer_total_current_limit * power_scale;
    }
    else{
        if(power.chassis_power > 40.0)//warning_power
        {
            float power_scale;

            if(power.chassis_power < 80.0)//power_limit
            {

                power_scale = (80.0 - power.chassis_power) / (80.0 - 40.0);

            }
            else{
                power_scale = 0.0f;
            }

            total_current_limit = buffer_total_current_limit + power_total_current_limit * power_scale;
        }
        else{
            total_current_limit = buffer_total_current_limit + power_total_current_limit;
        }
    }

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gary_chassis::ChassisPowercontrol)














































/*
#include"rclcpp/rclcpp.hpp"
#include "gary_msgs/msg/power_heat.hpp"
#include <chrono>
#include "rclcpp_components/register_node_macro.hpp"
#include "chassis_powercontrol/hpp"


class power_control_lifecyclenode: public rclcpp::Node{
public:
    explicit power_control_lifecyclenode(const std::string& node_name,bool intra_process_comms= false)
    :rclcpp_lifecycle::LifeCyclenode(node_name,
                                     rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)){

    }
};

//#include "rclcpp_components/register_node_macro.hpp"


#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "lifecycle_msgs/msg/transition.hpp"
#include "std_msgs/msg/string.hpp"
gary_msgs::msg::PowerHeat power;
void time_callback();
void power_callback();
using namespace std::chrono_literals;

class my_lifecyclenode : public rclcpp_lifecycle::LifecycleNode {





public:
    // 构造函数
    // lifecyclenode 的构造函数都有相同的参数
    explicit my_lifecyclenode(const std::string& node_name, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(node_name,
                                              rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {


    }
    void func() {


        std::cout << "in func" << std::endl; }

    // on_configure回调函数会在lifecyclenode进入configuring状态时被调用
    // 根据返回值的不同，节点会进入inactive或者停留在unconfigured
    // RANSITION_CALLBACK_SUCCESS transitions to "inactive"
    // RANSITION_CALLBACK_FAILURE transitions to "unconfigured"
    // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State&) {



        RCLCPP_INFO(get_logger(), "on_configure() is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // 没啥好说的，参考on_configure回调函数和前文
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State&) {



        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
        std::this_thread::sleep_for(2s);
        power_callback();
        time_callback();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    //
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State&) {
S


        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    //
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State&) {



        // 做一些publisher和timer的reset等


        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& state) {



        RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
};


class PowerControl : public rclcpp::Node
{
public:
    PowerControl() : Node("power_control")
    {
        power_sub_ = create_subscription<gary_msgs::msg::PowerHeat>("power",rclcpp::SystemDefaultsQoS(),
                                                                    std::bind(&PowerControl::power_callback,this,std::placeholders::_1));
        timer_ = this->create_wall_timer(500ms,std::bind(&PowerControl::time_callback,this));
    }
private:
void power_callback(gary_msgs::msg::PowerHeat::SharedPtr msg)
{
    power = *msg;
}

void time_callback()
{

    const float warning_power_buffer = 50.0,warning_power = 40.0;
    const float buffer_total_current_limit = 16000.0 , power_total_current_limit = 20000.0;
    float total_current_limit = 0.0;
    if(power.chassis_power_buffer < warning_power_buffer)
    {
        float power_scale;
        if(power.chassis_power_buffer > 5.0)
        {
            power_scale = power.chassis_power_buffer / warning_power_buffer;
        }
        else{
            power_scale = 5.0f / warning_power_buffer;
        }
        total_current_limit = buffer_total_current_limit * power_scale;
    }
    else{
        if(power.chassis_power > 40.0)//warning_power
        {
            float power_scale;

            if(power.chassis_power < 80.0)//power_limit
            {

                power_scale = (80.0 - power.chassis_power) / (80.0 - 40.0);

            }
            else{
                power_scale = 0.0f;
            }

            total_current_limit = buffer_total_current_limit + power_total_current_limit * power_scale;
        }
        else{
            total_current_limit = buffer_total_current_limit + power_total_current_limit;
        }
    }

}
rclcpp::Subscription<gary_msgs::msg::PowerHeat>::SharedPtr power_sub_;
rclcpp::TimerBase::SharedPtr timer_;


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PowerControl>());
    rclcpp::shutdown();
    return 0;
}



class ChassisPowercontrol;



   #include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(composition::ChassisPowercontrol)
*/