Testing server package ros_gazebo_v1 independently necessarily involves a catkin make instruction inside your workspace. Additionally, the route for the testing image file must be updated in src/drone.cc. Once this is done, two terminals must be opened for executing the following commands (one for each terminal):

- Basic user interface:
    ```sh
    $ rosrun ros_gazebo_v1 interface
    ```
    
- Gazebo simulation:
    ```sh
    $ roslaunch ros_gazebo_v1 environment_gazebo.launch
    ```

Doing so will result in an order to a box simulating the drone for sending the image to the box acting as server. It will be processed and the balls in the court will be ordered attending proximity criteria to the reference point (0,0). When the execution finishes correctly, messages communicating the end of the cycle will appear.
