#include <iostream>
#include "rceti_continuum/Continuum.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <termios.h> // For terminal input settings
#include <unistd.h>  // For read()

// Function to configure terminal for non-blocking input
void configureTerminal(struct termios &initial_settings, struct termios &new_settings)
{
    tcgetattr(0, &initial_settings);//current settings
    new_settings = initial_settings;//copy to new
    new_settings.c_lflag &= ~ICANON;//process characters as soon as they are typed
    new_settings.c_lflag &= ~ECHO;//no echo
    new_settings.c_cc[VMIN] = 1;//minimum number of characters to read
    new_settings.c_cc[VTIME] = 0;//no timeout
    tcsetattr(0, TCSANOW, &new_settings);//apply
}

// Function to restore terminal settings
void restoreTerminal(struct termios &initial_settings)
{
    tcsetattr(0, TCSANOW, &initial_settings);//restore original settings
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("continuum_core");
    Continuum robot(node);

    RCLCPP_INFO(node->get_logger(), "Number of segments: %d", robot.numberOfSegments);

    // Set the base pose
    tf2::Quaternion q;
    q.setRPY(0.0, 90 * PI / 180, 0.0);
    double base_x = node->declare_parameter("base_pose.x", 0.0);
    double base_y = node->declare_parameter("base_pose.y", 0.0);
    double base_z = node->declare_parameter("base_pose.z", 0.0);
    robot.setSegmentBasePose(0, tf2::Vector3(base_x, base_y, base_z), q);

    // Assign parameters for each section
    for (int i = 0; i <= robot.numberOfSegments - 1; i++)
    {
        robot.addSegment(i, 5 * (i + 1), 2, .3); // SegID, Length, noOfSegments, radius of disk
        robot.setSegmentShape(0, 0.0001, 0);     // SegID, Kappa, Phi
    }

    // Configure terminal for non-blocking input
    struct termios initial_settings, new_settings;
    configureTerminal(initial_settings, new_settings);

    RCLCPP_INFO(node->get_logger(), "w/s: Increase/Decrease curvature (kappa)");
    RCLCPP_INFO(node->get_logger(), "a/d: Rotate left/right (phi)");
    RCLCPP_INFO(node->get_logger(), "Q: Quit");

    double kappa = 0.0;//curve
    double phi = 0.0;//rotation

    while (rclcpp::ok())
    {
        char input = getchar();//read user chars

        if (input != EOF) // Check if input was captured
        {
            RCLCPP_INFO(node->get_logger(), "input: %c", input);

            if (input == 'w')
            {
                kappa += 0.01;//increase curve
            }
            else if (input == 's')
            {
                kappa -= 0.01;//decrease curve
            }
            else if (input == 'a')
            {
                phi += PI / 36;//rotate left
            }
            else if (input == 'd')
            {
                phi -= PI / 36;//rotate right
            }
            else if (input == 'q')
            {
                RCLCPP_INFO(node->get_logger(), "quit");
                break;//exit
            }

            //set and update shape using continuum.cpp
            robot.setSegmentShape(0, kappa, phi); 
            robot.update();                      

            RCLCPP_INFO(node->get_logger(), "updated shape: kappa = %f, phi = %f", kappa, phi);
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "no input detected");
        }

        //rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    // Restore terminal settings
    restoreTerminal(initial_settings);

    rclcpp::shutdown();
    return 0;
}
