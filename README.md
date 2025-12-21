## Pre-Requisites
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
* [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
* [rosdep](https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Deb-Package.html#initialize-rosdep)

## Initial Setup
1. Create a workspace
    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    ```
2. Clone the repo
    ```bash
    git clone https://github.com/ERAU-Student-Projects-ME/System_Diagnostic.git -b develop
    ```
3. Check all dependencies
   ```bash
   cd ..
   rosdep -i install --from-path src --rosdistro humble -y
   ```
4. Build the packages
    ```bash
    colcon build --symlink-install --merge-install
    ```
   **Note:** If you run into error while building, open the `.bashrc` script and confirm that, the ROS2 bash file is sourced.
   ```bash
   gedit ~/.bashrc
   # Scroll down in the script and look for the following line:
   # source /opt/ros/humble/setup.bash
   ``` 
   If the ROS2 bash file is not sourced, add it and save it. Close the `.bashrc` script, and any open terminal. Open a new terminal and retry building the packages.

5. Source the workspace
    ```bash
    echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
    ```
## Run the system health monitor
In a new terminal `Ctrl + Alt + T`, run the following command
```bash
ros2 launch minion_diagnostic minion_diagnostic.launch.py
```

