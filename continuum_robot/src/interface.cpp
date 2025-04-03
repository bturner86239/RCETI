#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <termios.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PI 3.1415926

using namespace visualization_msgs;
using namespace std;
struct termios initial_settings,
               new_settings;

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
tf2::Quaternion directionQauat;

// %Tag(vars)%
// %EndTag(vars)%
// %Tag(Box)%

//Marker makeBox( InteractiveMarker &msg )

//{
  //Marker marker;

 // marker.type = Marker::ARROW;
 visualization_msgs::msg::Marker makeBox(visualization_msgs::msg::InteractiveMarker &msg)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 0.2;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

/*
InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back( control );

  return msg.controls.back();
}
*/
visualization_msgs::msg::InteractiveMarkerControl& makeBoxControl(visualization_msgs::msg::InteractiveMarker &msg)
{
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);
    return msg.controls.back();
}



void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
    std::ostringstream s;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s: pose changed\nposition = %f, %f, %f\norientation = %f, %f, %f, %f\nframe: %s time: %d sec, %d nsec",
                s.str().c_str(),
                feedback->pose.position.x,
                feedback->pose.position.y,
                feedback->pose.position.z,
                feedback->pose.orientation.w,
                feedback->pose.orientation.x,
                feedback->pose.orientation.y,
                feedback->pose.orientation.z,
                feedback->header.frame_id.c_str(),
                feedback->header.stamp.sec,
                feedback->header.stamp.nanosec);

    server->applyChanges();
}

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
// %Tag(6DOF)%

void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf2::Vector3& position, bool show_6dof )
{
  //InteractiveMarker int_marker;
  visualization_msgs::msg::InteractiveMarker int_marker;

  int_marker.header.frame_id = "moving_frame";
  tf2::toMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "robotHead";
  int_marker.description = "robotHead";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  //InteractiveMarkerControl control;
  visualization_msgs::msg::InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    //control.orientation_mode = InteractiveMarkerControl::FIXED;
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;

  }

  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

    int_marker.controls.push_back(control);
    control.name = "move_x";
    //control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

    int_marker.controls.push_back(control);
    control.name = "move_z";
    //control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

    int_marker.controls.push_back(control);
    control.name = "move_y";
    //control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
 }
// %EndTag(6DOF)%
// %Tag(frameCallback)%

void frameCallback(std::shared_ptr<rclcpp::Node> node)
{
    static double thetaPitch = 0;
    static double thetaYaw = 0;
    static uint32_t counter = 0;

    //static tf2_ros::TransformBroadcaster br;
    static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    br = std::make_shared<tf2_ros::TransformBroadcaster>(node);


    static geometry_msgs::msg::TransformStamped t;

    rclcpp::Time time = rclcpp::Clock().now();
    int n = getchar();

    if (n == '\033') { // if the first value is esc
        getchar(); // skip the [
        switch(getchar()) { // the real value
            case 'A':
                cout << "ARROW UP" << endl; // code for arrow up
                thetaPitch -= 90;
                break;
            case 'B':
                cout << "ARROW DOWN" << endl; // code for down up
                thetaPitch += 90;
                break;
            case 'C':
                cout << "ARROW right" << endl; // code for arrow right
                thetaYaw -= 90;
                break;
            case 'D':
                cout << "ARROW left" << endl; // code for arrow left
                thetaYaw += 90;
                break;
        }
        directionQauat.setRPY(0.0, (thetaPitch * PI) / 180, (thetaYaw * PI) / 180);
    }
    cout << "p = [" << t.transform.translation.x << ", " << t.transform.translation.y << ", " << t.transform.translation.z << "]" << endl;

    t.transform.rotation = tf2::toMsg(directionQauat);
    //t.transform.rotation = tf2::Quaternion(directionQauat.x(), directionQauat.y(), directionQauat.z(), directionQauat.w());

    t.header.stamp = time;
    t.header.frame_id = "base_link";
    t.child_frame_id = "moving_frame";
    //br.sendTransform(t);
    br->sendTransform(t);

    //t.transform.translation.x += tf2::Matrix3x3(directionQauat) * tf2::Vector3((float(1) / 140.0) * 2.0, 0, 0);
    tf2::Vector3 translation_offset = tf2::Matrix3x3(directionQauat) * tf2::Vector3((float(1) / 140.0) * 2.0, 0, 0);

t.transform.translation.x += translation_offset.x();
t.transform.translation.y += translation_offset.y();
t.transform.translation.z += translation_offset.z();
}

// %EndTag(frameCallback)%
// %Tag(Moving)%

void makeMovingMarker( const tf2::Vector3& position )
{
  //InteractiveMarker int_marker;
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "moving_frame";
  tf2::toMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  //InteractiveMarkerControl control;
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  //control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

  int_marker.controls.push_back(control);

  //control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

/* int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("interface");
    rclcpp::Rate r(1);
    directionQauat = tf2::Quaternion(0, 0, 0, 1);
    // Keyboard
    tcgetattr(0, &initial_settings);

    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);

    // create a timer to update the published transforms
    //auto frame_timer = node->create_wall_timer(std::chrono::milliseconds(50), frameCallback);
    //auto frame_timer = node->create_wall_timer(std::chrono::milliseconds(50), std::bind(frameCallback, node));
    auto frame_timer = node->create_wall_timer(
      std::chrono::milliseconds(50),
      [node]() { frameCallback(node); }
  );


    auto robotShapePublisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("robot_shape", 1);
    server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", node, rclcpp::QoS(10).transient_local()));
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    tf2::Vector3 position;
    position = tf2::Vector3(0, 0, 0);
    make6DofMarker(false, visualization_msgs::msg::InteractiveMarkerControl::NONE, position, false);
    server->applyChanges();

    rclcpp::spin(node);

    server.reset();
    tcsetattr(0, TCSANOW, &initial_settings);
    return 0;
}
*/

