#include "path_manager/path_manager.hpp"


int main(int argc, char * argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ); // Ensures synchronized stdout when launched from ROS launch files.

    rclcpp::init(argc, argv);

    auto path_manager_node = std::make_shared<PathManager>("path_manager_node");
    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(path_manager_node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}

