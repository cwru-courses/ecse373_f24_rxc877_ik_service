# ecse373_f24_rxc877_ik_service
# ik_service

The `ik_service` package provides a ROS service for calculating inverse kinematics for the UR10 manipulator. It allows users to send a target pose and receive the corresponding joint angles as a response.

## Prerequisites

Ensure you have the following dependencies installed 

- **ROS Noetic**
- [ur_kinematics](https://github.com/ros-industrial/universal_robot/tree/noetic-devel/ur_kinematics)
- [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)


## Instalation 

1. Clone the repository:
    ```bash
    git clone git@github.com:cwru-courses/ecse373_f24_rxc877_ik_service.git

    
2. Open a terminal and navigate to the ROS workspace:
    ```bash
    cd ~/ik_ws
    ```

3. Compile the workspace:
    ```bash
    catkin_make
    ```

4. Source your workspace:
    ```bash
    source devel/setup.bash
    ```
       
### Steps to Launch the nodes

1. Launch the nodes 
    ```bash
    roslaunch ik_service ik_service.launch
    ```



### Example Output

When running the above command, you should see output similar to the following:
```bash
[ INFO] [1731083582.322400622]: waitForService: Service [/calculate_ik] has not been advertised, waiting...
[ INFO] [1731083582.335877675]: Service ready 
[INFO] [1731083582.346701309]: The ik_service has been called 
[ INFO] [1731083582.346857547]: Call to ik_service returned [8] solutions 
[ INFO] [1731083582.346876234]: solution 0:{ 5.949125, 4.769161, 1.772193, 1.312628, 1.570796, 1.236736} 
... 
[ INFO] [1731083582.347924473]: solution 7:{ 4.160967, 4.309900, 4.942447, 1.743228, 4.712389, 2.590171}
[ik_client_node-3] process has finished cleanly
log file: /home/rosalia/.ros/log/1f17f866-9def-11ef-9ca0-ed61b4cde90b/ik_client_node-3*.log
```

### Explanation Output 
First  the parmeter ```wait_for_service```has been configured to wait for the client to launch when ```/calculate_ik``` is ready. 

Then the Service is called correctky and 8 solutions of the inverse kinematic have to be send to the client.

Finally the client is finished and closes correctly 

Aclaration: The client node is runned two times so you have twice the responces, this can be change in the .cpp changing the loop. 

## File Structure
```bash
The package structure is as follows:
ik_service/
├── CMakeLists.txt
├── package.xml
├── launch
│   └── ik_service.launch
├── msg
│   └── JointSolutions.msg
├── src
│   ├── ik_service.cpp
│   └── ik_client.cpp
├── srv
    └── PoseIK.srv
```

### Description of Files
- **CMakeLists.txt**: Configuration file for building the package.
- **package.xml**: Defines package dependencies.
- **launch/ik_service.launch**: Launch file to run both the service and client nodes.
- **msg/JointSolution.msg**: Custom message definitions (if needed).
- **src/ik_service.cpp**: Service node implementation for inverse kinematics.
- **src/ik_client.cpp**: Client node that calls the IK service.
- **srv/PoseIK.srv**: Service definition file for IK requests.



## Other documentation 
You can also visit the following repositories for more resources:

    cwru_ariac_2019: https://github.com/cwru-courses/cwru_ariac_2019
    ecse_373_ariac: https://github.com/cwru-courses/ecse_373_ariac
