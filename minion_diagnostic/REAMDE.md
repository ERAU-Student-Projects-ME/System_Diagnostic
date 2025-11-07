# Overview

The `minion_diagnostic` is a ROS2 C++ - Python package, meaning it can have nodes written in both C++ and python.
It contains the main system diagnostic node and GUI plugin, source code.

---

## Adding a Python node

To add a python ROS2 node:
1. Add/Create your python source code inside the `minion_diagnostic` directory.
2. At the top of your python node, make sure to add the following Shebang - `#!/usr/bin/env python3`
3. Right click on the file, go to the properties, and make it an executable. Or open a terminal in the current
   directory (`minion_diagnostic`), and type the following command to give the file, executable rights:
   
   ```bash
   chmod +x <name of the python file>
   ```
5. Open `package.xml` and add all python modules as a dependency. Example -
   
   ```xml
   <depend>python3-pyqtgraph</depend>
   ```
6. Open `CMakeList.txt` and add the new node inside the following tag:
   
   ```cmake
   ament_python_install_package(${PROJECT_NAME})
   # Install Python executables
   install(PROGRAMS
     minion_diagnostic/system_health_monitor.py
     minion_diagnostic/systeminfo.py

     # Add the new node here
   
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

 ## Build and Run
 1. Navigate back to root of your workspace, and build the package
    
    ```bash
    colcon build --merge-install --symlink-install
    ```
 2. Open a new terminal to source the new build, and then run the program:
 
    ```bash
    ros2 run minion_diagnostic <name of the python file>
    ```
    **Note:** The `.py` extension must be included in the name of the python file.

