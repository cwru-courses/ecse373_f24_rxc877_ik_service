#include "ros/ros.h"
#include "ik_service/PoseIK.h"
#include "geometry_msgs/Pose.h"

void callIKService(ros::ServiceClient &client, const geometry_msgs::Pose &pose) {
    ik_service::PoseIK ik_pose;
    ik_pose.request.target_pose = pose;

    if (client.call(ik_pose)) {
       
        ROS_INFO("Call to ik_service returned [%i] solutions", ik_pose.response.num_solutions);
        // Show the solutions
        if (ik_pose.response.num_solutions > 0) {
            for (int i = 0; i < ik_pose.response.num_solutions; ++i) {
            ROS_INFO("solution %i:{ %f, %f, %f, %f, %f, %f}",i,ik_pose.response.joint_solutions[i].joint_angles[0],ik_pose.response.joint_solutions[i].joint_angles[1],
            ik_pose.response.joint_solutions[i].joint_angles[2],ik_pose.response.joint_solutions[i].joint_angles[3],ik_pose.response.joint_solutions[i].joint_angles[4],
            ik_pose.response.joint_solutions[i].joint_angles[5]);}
        }
    } else {
        ROS_ERROR("Failed to call service ik_service");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_client");
    ros::NodeHandle n;
    
    ros::ServiceClient client = n.serviceClient<ik_service::PoseIK>("calculate_ik");
    
    while (!client.waitForExistence(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the service to be ready ");
    }
    
    // Defining the pose initial 
    geometry_msgs::Pose pose1;
    pose1.position.x = 0.5;
    pose1.position.y = 0.0;
    pose1.position.z = 0.5;

    geometry_msgs::Pose pose2;
    pose2.position.x = 0.5;
    pose2.position.y = 0.5;
    pose2.position.z = 0.5;

    //calling the service
    callIKService(client, pose1);
    callIKService(client, pose2);

    return 0;
}