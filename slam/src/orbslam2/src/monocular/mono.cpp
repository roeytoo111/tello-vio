#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"

#include <unistd.h>
#include <limits.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if(argc < 3)
    {
        cerr << endl << "Usage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << endl;        
        rclcpp::shutdown();
        return 1;
    }

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR);
    auto node = std::make_shared<MonocularSlamNode>(&SLAM, argv[1], argv[2]);

    rclcpp::spin(node);
    
    // Stop threads and persist trajectory before tearing down ROS.
    char cwd[PATH_MAX];
    const std::string out_path =
        (getcwd(cwd, sizeof(cwd)) != nullptr)
            ? (std::string(cwd) + "/KeyFrameTrajectory.txt")
            : std::string("KeyFrameTrajectory.txt");
    node->ShutdownAndSave(out_path);

    rclcpp::shutdown();

    return 0;
}




