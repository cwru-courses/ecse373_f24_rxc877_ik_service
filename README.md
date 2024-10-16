# ecse373_f24_rxc877_ik_service
# ik_service

The `ik_service` package provides a ROS service for calculating inverse kinematics for the UR10 manipulator. It allows users to send a target pose and receive the corresponding joint angles as a response.

## Package Description

This package is designed to interact with the UR10 manipulator, utilizing the `ur_kinematics` library to compute the necessary joint angles to achieve a desired position and orientation.

## Launching the Service

To launch the service, ensure that your ROS environment is properly set up and that the `ik_service` package is compiled.

### Steps to Launch the Service

1. Open a terminal and navigate to the ROS workspace:
    ```bash
    cd ~/ik_ws
    ```

2. Compile the workspace:
    ```bash
    catkin_make
    ```
3. In a new terminal run the roscore
   
4. Start the service node:
    ```bash
    rosrun ik_service ik_service

5. Start the client node:
   ```bash
   rosrun ik_service ik_client 

## Verifying the Node Works as Intended

To verify that the `ik_service` node works as expected, use the service client. When running the client, you should see output similar to the following:
In the service terminal:
```bash
Service ready
The ik_service has been called
```
In the client terminal:
```bash
Call to ik_service returned [2] solutions
solution 0:{ 0.0, -0.785, 1.570, 0.0, 1.570, 0.0 }
solution 1:{ 0.0, 0.785, 1.570, 0.0, 1.570, 0.0 }

```

## Other documentation 
You can also visit the following repositories for more resources:

    cwru_ariac_2019: https://github.com/cwru-courses/cwru_ariac_2019
    ecse_373_ariac: https://github.com/cwru-courses/ecse_373_ariac
