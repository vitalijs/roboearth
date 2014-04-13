
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "re_msgs/DetectedObject.h"
#include <string.h>
#include <iostream>

#include "re_kinect_object_detector/DetectionResult.h"



#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/ObjectEvidence.h"

#include "problib/conversions.h"


// Publisher used to send evidence to world model
ros::Publisher world_evidence_publisher_;





void addEvidence(wire_msgs::WorldEvidence& world_evidence, double x, double y, double z, const std::string& class_label, const std::string& color) {
	wire_msgs::ObjectEvidence obj_evidence;

	// Set the continuous position property
	wire_msgs::Property posProp;
	posProp.attribute = "position";

	// Set position (x,y,z), set the covariance matrix as 0.005*identity_matrix
	pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.0005, 0.0005, 0.0005)), posProp.pdf);
	obj_evidence.properties.push_back(posProp);

	// Set the continuous orientation property
	wire_msgs::Property oriProp;
	oriProp.attribute = "orientation";

	// Set the orientation (0,0,0,1), with covariance matrix 0.01*identity_matrix
	pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector4(0, 0, 0, 1), pbl::Matrix4(0.01, 0.01, 0.01, 0.01)), oriProp.pdf);
	obj_evidence.properties.push_back(oriProp);

	// Set the discrete class label property
	wire_msgs::Property classProp;
	classProp.attribute = "class_label";
	pbl::PMF classPMF;

	// Probability of the class label is 0.7
	classPMF.setProbability(class_label, 0.7);
	pbl::PDFtoMsg(classPMF, classProp.pdf);
	obj_evidence.properties.push_back(classProp);

	// Set the discrete color property with a probability of 0.9
	wire_msgs::Property colorProp;
	colorProp.attribute = "color";
	pbl::PMF colorPMF;

	// The probability of the detected color is 0.9
	colorPMF.setProbability(color, 0.1);
	pbl::PDFtoMsg(colorPMF, colorProp.pdf);
	obj_evidence.properties.push_back(colorProp);

	// Add all properties to the array
	world_evidence.object_evidence.push_back(obj_evidence);
}

//
//void generateEvidence() {
//
//	// Create world evidence message
//	wire_msgs::WorldEvidence world_evidence;
//
//	// Set header
//	world_evidence.header.stamp = ros::Time::now();
//	world_evidence.header.frame_id = "/map";
//
//	// Add evidence
//	addEvidence(world_evidence, 2, 2.2, 3, "mug", "red");
//
//	// Publish results
//	world_evidence_publisher_.publish(world_evidence);
//	ROS_INFO("Published world evidence with size %d", world_evidence.object_evidence.size());
//
//}


void chatterCallback( re_kinect_object_detector::DetectionResultConstPtr msg)
{
//	ROS_INFO("Object detected");
	if(msg->ObjectNames.size()>0){
		ROS_INFO("ObjectNames.size() = %d\n\tframe_id=%s", msg->ObjectNames.size(), msg->Image.header.frame_id.c_str());
		// Over all recognized Object names
		for(unsigned int i=0; i<msg->ObjectNames.size(); i++){
			ROS_INFO("name[%d] = %s", i, msg->ObjectNames[i].c_str());
//			std::cout<<"name["<<i<<"] = "<<msg->ObjectNames[i]<<std::endl;
		}
		int pSize=msg->Detections.size();
		ROS_INFO("Detections.size() = %d", pSize);
		for(unsigned int p=0; p<pSize; p++){

//			Inactive Data

			ROS_INFO("Detections.points2d.size() = %d", msg->Detections[p].points2d.size());
//			ROS_INFO("points2d[%d] Point(%f, %f)", p,
//					msg->Detections[p].points2d[0].x,
//					msg->Detections[p].points2d[0].y);

			ROS_INFO("Detections.points3d.size() = %d", msg->Detections[p].points3d.size());
			ROS_INFO("points3d[%d] Point(%f, %f, %f)", p,
					msg->Detections[p].points3d[0].x,
					msg->Detections[p].points3d[0].y,
					msg->Detections[p].points3d[0].z);


			// Create world evidence message
			wire_msgs::WorldEvidence world_evidence;

			// Set header
			world_evidence.header.stamp = ros::Time::now();
//			world_evidence.header.frame_id = "/map";
			world_evidence.header.frame_id = msg->Image.header.frame_id;
			double x,y,z;
			for( int i=0; i< msg->Detections[p].points3d.size();i++){
				x += msg->Detections[p].points3d[i].x;
				y += msg->Detections[p].points3d[i].y;
				z += msg->Detections[p].points3d[i].z;
			}
			x = x / msg->Detections[p].points3d.size();
			y = y / msg->Detections[p].points3d.size();
			z = z / msg->Detections[p].points3d.size();

			ROS_INFO("points3d[%d] Object Point(%f, %f, %f)", p, x, y, z);
			// Add evidence
			addEvidence(world_evidence, x, y, z, msg->ObjectNames[p].c_str(), "red");

			// Publish results
			world_evidence_publisher_.publish(world_evidence);
			ROS_INFO("Published world evidence with size %d", world_evidence.object_evidence.size());




//			 Inactive Data

//			ROS_INFO("Pose[%d] Point(%f, %f, %f)   Orientation(%f, %f, %f, %f)", p,
//					msg->Detections[p].pose.position.x,
//					msg->Detections[p].pose.position.y,
//					msg->Detections[p].pose.position.z,
//					msg->Detections[p].pose.orientation.x,
//					msg->Detections[p].pose.orientation.y,
//					msg->Detections[p].pose.orientation.z,
//					msg->Detections[p].pose.orientation.w);
		}
	}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "re2wire_bridge");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/re_kinect/detection_results", 1000, chatterCallback);


  // Publisher
  world_evidence_publisher_ = n.advertise<wire_msgs::WorldEvidence>("/world_evidence", 100);

//
//  // Publish with 3 Hz
//  ros::Rate r(3.0);
//
//  while (ros::ok()) {
//    generateEvidence();
//    r.sleep();
//  }
//


  ros::spin();

  return 0;
}
