#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>

int main(int argc, char** argv)
{
    // 1. 初始化 ROS 2 节点
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    // MoveIt 2 需要自动声明参数的权限
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("follow_point_node", node_options);

    // 启动一个单线程执行器处理回调
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    RCLCPP_INFO(node->get_logger(), "正在连接 MoveIt 2 大脑...");

    // 2. 创建 MoveGroup 接口（这里的 "arm" 必须和你之前在 Setup Assistant 里填的组名一致）
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "arm");

    // 3. 设定目标姿态并使用“近似求解” (专门对付自由度不足的机械臂)
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0; 
    
    // 设定一个绝对安全、不碰地、不碰底座的半空位置
    target_pose.position.x = 0.3; // 往前 30 厘米
    target_pose.position.y = 0.0; // 正前方
    target_pose.position.z = 0.8; // 离地 80 厘米 (非常关键，防止砸到自己)

    RCLCPP_INFO(node->get_logger(), "设定安全目标位置: X:%.2f, Y:%.2f, Z:%.2f", 
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    // 使用 Approximate 求解，它会尽最大努力去靠拢这个点，而不会因为姿态问题死锁
    move_group.setApproximateJointValueTarget(target_pose);

    // 4. 进行运动轨迹规划
    RCLCPP_INFO(node->get_logger(), "开始计算逆运动学并规划轨迹...");
    MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 5. 如果规划成功，则执行移动
    if (success) {
        RCLCPP_INFO(node->get_logger(), "规划成功！开始移动...");
        move_group.move();
    } else {
        RCLCPP_ERROR(node->get_logger(), "规划失败！目标点可能超出了机械臂的长度范围，或者会发生碰撞。");
    }

    // 关闭节点
    rclcpp::shutdown();
    return 0;
}