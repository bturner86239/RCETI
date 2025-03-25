/*
 * Continuum.h
 *
 *  Created on: Apr 1, 2017
 *      Author: haitham
 */
//
#ifndef CONTINUUM_ROBOT_INCLUDE_CONTINUUM_ROBOT_CONTINUUM_H_
#define CONTINUUM_ROBOT_INCLUDE_CONTINUUM_ROBOT_CONTINUUM_H_
#include "rclcpp/rclcpp.hpp"

#include <math.h>
#include "std_msgs/msg/string.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"


#include <tf2_ros/transform_broadcaster.h>
#include <stdlib.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

#include <termios.h>

using namespace std;
#define PI 3.1415926
#define RESOLUTION 100
#define INTERFACE 0
#define delay 1
#define HEAD 1
#define TAIL 0
#define FLIPPED 1
#define NORMAL 0
#define UPDATERATE 50
class Continuum {
private:
	 tf2::Transform* endEffectorPose;
	 tf2::Transform* basePose;
	 tf2::Transform** segTFFrame; // array of array segTFFrame[segID][diskNo]
	 std::shared_ptr<tf2_ros::TransformBroadcaster> segTFBroadcaster;

	 visualization_msgs::msg::MarkerArray* cableMarkers;
	 visualization_msgs::msg::MarkerArray headMarkers;
	// rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cablePublisher;
	 std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> cablePublisher;

	 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr headPublisher;

	 rclcpp::TimerBase::SharedPtr frame_timer;

	 bool hasHead;
	 int headDisks;
	 double headLength;
	 double* arrayOfKappa;
	 double* arrayOfPhi;
	 double* segmentLength;
	 int* noOfDisks;
	 int* segmentMode;
	 double* segKappa;
	 double* segPhi;
	 double headPhi;
	 int headMode;
	 double headKappa;
	 struct termios initial_settings,
	                new_settings;
	 int rateOfUpdate;
	 //-------------------------------------------------------------
	 std_msgs::msg::String robotName;
	 ofstream robotURDFfile;
	 void createURDF(int segID, double length, int n_disks, double radius);
	 void initCableMarker(int segID);
	 tf2::Quaternion getDiskQuaternion(int segID, int diskID);
	 tf2::Quaternion getHeadQuaternion(int diskID);

	 tf2::Vector3 getDiskPosition(int segID, int i);
	 void timerScanning();
public:
	Continuum(std::shared_ptr<rclcpp::Node> node);
	 int numberOfSegments;

	void addSegment(int segID, double length, int n_disks, double radius);

	void setSegmentBasePose(int segID, tf2::Vector3 basePos, tf2::Quaternion baseRot);
	void setSegmentShape(int segID, double kappa, double phi);
	void update(void);
	void addHead(double len, int disks, double rad);
	void setHeadParameters(double headKap, double headPhi, int MODE);


	virtual ~Continuum();
};

#endif /* CONTINUUM_ROBOT_INCLUDE_CONTINUUM_ROBOT_CONTINUUM_H_ */
