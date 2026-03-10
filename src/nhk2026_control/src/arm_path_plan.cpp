#include "arm_path_plan.hpp"

ArmPathPlan::ArmPathPlan()
: rclcpp::Node("arm_path_plan")
{
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ArmPathPlan> node = std::make_shared<ArmPathPlan>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}