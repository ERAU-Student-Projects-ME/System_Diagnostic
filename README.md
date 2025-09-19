## Pre-Requisites
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
* [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
* [rosdep](https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Deb-Package.html#initialize-rosdep)

## Initial Setuo
1. Create a workspace
    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    ```
2. Clone the repo
    ```bash
    git clone https://github.com/ERAU-Student-Projects-ME/System_Diagnostic.git -b develop
    ```  
3. Build the packages
    ```bash
    cd ..
    colcon build --symlink-install --merge-install
    ```
4. Source the workspace
    ```bash
    echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
    ```
## Run the system health monitor
In a new terminal `Ctrl + Alt + T`, run the following command
```bash
ros2 launch minion_diagnostic minion_diagnostic.launch
```

