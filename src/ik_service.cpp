#include "ros/ros.h"
#include "ik_service/PoseIK.h"
#include "ur_kinematics/ur_kin.h"

bool calculateIK(ik_service::PoseIK::Request &req, ik_service::PoseIK::Response &res) {
    ROS_INFO("The ik_service has been called");

    double q_sols[8][6]; // Array para los ángulos de las articulaciones
    double q6_des = 0.0; // Valor deseado para la última articulación
    

    //Convert the pose in a 4x4 matrix T
    double T[4][4] = { {0, -1, 0, req.target_pose.position.x},
                       {0, 0, 1, req.target_pose.position.y},
                       {-1, 0, 0, req.target_pose.position.z},
                       {0, 0, 0, 1} }; 
//   double T[4][4] = { {0, -1, 0, 0.5},
//                        {0, 0, 1, 0.0},
//                        {-1, 0, 0, 0.5},
//                        {0, 0, 0, 1} };

    // Calling the inverse pose
    if (ur_kinematics::inverse(&T[0][0], &q_sols[0][0], q6_des)) {
        res.num_solutions = ur_kinematics::inverse(&T[0][0], &q_sols[0][0], 0.0); // Asumiendo que siempre habrá una solución
        for (int i = 0; i < res.num_solutions; ++i) {
        res.joint_solutions[i].joint_angles[0] = q_sols[i][0];
        res.joint_solutions[i].joint_angles[1] = q_sols[i][1];
        res.joint_solutions[i].joint_angles[2] = q_sols[i][2];
        res.joint_solutions[i].joint_angles[3] = q_sols[i][3];
        res.joint_solutions[i].joint_angles[4] = q_sols[i][4];
        res.joint_solutions[i].joint_angles[5] = q_sols[i][5];
        
        }

    } else {
        ROS_INFO("There's no solution");
        res.num_solutions = 0; // No solution
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_service");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("calculate_ik", calculateIK);
    ROS_INFO("Service ready ");
    ros::spin();

    return 0;
}

