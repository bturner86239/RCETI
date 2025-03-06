#include <iostream>
#include "continuum_robot/Continuum.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    //auto node = rclcpp::Node::make_shared("continuum_core");
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("continuum_core");
    Continuum robot(node);
    //Continuum robot; // create an object robot with three sections

    RCLCPP_INFO(node->get_logger(), "Number of segments: %d", robot.numberOfSegments);
    // set the base Pose
    //robot.setSegmentBasePose(0, tf2::Vector3(0, 0, 0), tf2::Quaternion(0.0, 90 * PI / 180, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 90 * PI / 180, 0.0);
    robot.setSegmentBasePose(0, tf2::Vector3(0, 0, 0), q);




    // Assign the parameters for each section
    for (int i = 0; i <= robot.numberOfSegments; i++)
    {
        robot.addSegment(i, 5 * (i + 1), 20, .3); // SegID , Length, noOfSegments, radius of disk
        robot.setSegmentShape(0, 0.0001, 0); // SegID , Kappa, Phi
    }

    // Moving demonstration:
    while (rclcpp::ok())
    {
        // set a pattern
        for (double i = 0.25; i >= -0.25; i -= 0.0051)
        {
            robot.setSegmentShape(0, i, PI / 4); // SegID , Kappa, Phi
            robot.update();
        }

        for (double i = -0.25; i <= 0.25; i += 0.00515)
        {
            robot.setSegmentShape(0, i, PI / 4); // SegID , Kappa, Phi
            robot.update();
        }

        for (double i = 0.25; i >= -0.25; i -= 0.0051)
        {
            robot.setSegmentShape(1, i, PI / 2); // SegID , Kappa, Phi
            robot.update();
        }

        for (double i = -0.25; i <= 0.25; i += 0.0051)
        {
            robot.setSegmentShape(1, i, PI / (2 * i)); // SegID , Kappa, Phi
            robot.update();
        }
    }
    rclcpp::shutdown();
    return 0;
}
